# %%
import argparse
import os
import pandas as pd
import numpy as np

# %% tags=["parameters"]
# declare a list tasks whose products you want to use as inputs
upstream = ['gt-prediction-association-*']
products = None
sample = os.environ.get("INPUT_BAG", "ex01_01")

# %%
GT_PREDICTION_ASSOCIATION_CSV='/data/out/gt_prediction_association.csv'
TIMESTAMPS_TO_KEEP='/data/out/timestamps_to_keep.csv'
MAX_MISSING_FRAMES=6
OUTPUT_CSV="/data/out/"
TARGET_WIDTH=0.5
CHALLENGE_NAME="people_tracker"
BAG_PATH = os.environ.get("BAG_REPO", "/data")

print(upstream)

# %%
def interpolate_data(gt_prediction_association_full_ts: pd.DataFrame, id: int):
    id_records = gt_prediction_association_full_ts
    id_records['interpolated'] = id_records['id'].isna()
    
    previous_valued_idx = 0

    for idx, id_record in id_records.iterrows():
        if not np.isnan(id_record['id']):
            if idx > previous_valued_idx + 1:
                current_x = id_record['x']
                current_y = id_record['y']
                current_timestamp = id_record['timestamp']

                previous_x = id_records.iloc[previous_valued_idx]['x']
                previous_y = id_records.iloc[previous_valued_idx]['y']
                previous_timestamp = id_records.iloc[previous_valued_idx]['timestamp']
                
                for i in range(1, idx - previous_valued_idx):
                    t = (id_records.iloc[previous_valued_idx + i]['timestamp'] - previous_timestamp) / (current_timestamp - previous_timestamp)
                    # id_records.iloc[previous_valued_idx + i]['id'] = id
                    # id_records.iloc[previous_valued_idx + i]['x'] = (current_x - previous_x) * t
                    # id_records.iloc[previous_valued_idx + i]['y'] = (current_y - previous_y) * t

                    id_records.at[previous_valued_idx + i,'id'] = id
                    id_records.at[previous_valued_idx + i, 'x'] = previous_x + (current_x - previous_x) * t
                    id_records.at[previous_valued_idx + i, 'y'] = previous_y + (current_y - previous_y) * t
            
            previous_valued_idx = idx
        
    return id_records

def compute_id_changes(id, id_records_full_ts, max_missing_frames, next_id):
    missing_frames = 0
    current_id = id
    id_changes = []

    for _, id_record in id_records_full_ts.iterrows():
        if not id_record['keep_y']:
            missing_frames += 1
        else:
            missing_frames = 0
        
        if missing_frames > max_missing_frames:
            print('id', current_id, 'replace with', next_id)
            id_changes.append((id_record['timestamp'], current_id, next_id))
            missing_frames = 0
            current_id = next_id
            next_id += 1

    return id_changes, next_id

def filter_by_id(gt_prediction_association, timestamp_to_keep, id: int):
    id_records = gt_prediction_association[gt_prediction_association['id'] == id]
    min_ts = id_records['gt_ts'].min()
    max_ts = id_records['gt_ts'].max()

    ts_interval = timestamp_to_keep[(timestamp_to_keep['timestamp'] >= min_ts) &
                                    (timestamp_to_keep['timestamp'] <= max_ts)]
    ts_interval = ts_interval[['timestamp', 'keep', 'number']]

    id_records_full_ts = pd.merge(
        left = id_records,
        right = ts_interval,
        left_on = 'gt_ts',
        right_on = 'timestamp',
        how = 'right'
    )

    return id_records_full_ts
    

def change_track_ids(input_gt_prediction_association: str, input_timestamp_to_keep: str, max_missing_frames: str, target_width: float):
    gt_prediction_association = pd.read_csv(input_gt_prediction_association, index_col=0)
    timestamp_to_keep = pd.read_csv(input_timestamp_to_keep)

    ids = gt_prediction_association.groupby(['id']).count().index
    next_id = ids.max() + 1

    for id in ids:
        id_records = gt_prediction_association[gt_prediction_association['id'] == id]

        id_records_full_ts = filter_by_id(gt_prediction_association, timestamp_to_keep, id)
        id_changes, next_id = compute_id_changes(id, id_records_full_ts, max_missing_frames, next_id) 

        for id_change in id_changes:
            ts, old_id, new_id = id_change
            gt_prediction_association.loc[
                (gt_prediction_association['id'] == old_id) & 
                (gt_prediction_association['gt_ts'] >= ts), 'id'] = new_id

        id_records_updated = interpolate_data(id_records_full_ts, id)
        interpolated_records = id_records_updated[id_records_updated['interpolated'] == True]
        interpolated_records = interpolated_records.drop(['keep_x', 'gt_ts', 'interpolated', 'number_x'], axis=1)
        interpolated_records = interpolated_records.rename(columns={ "keep_y": "keep", "timestamp": "gt_ts", "number_y": "number" })
        gt_prediction_association = pd.concat([gt_prediction_association, interpolated_records])
    
    gt_prediction_association = gt_prediction_association[gt_prediction_association['keep'] == True]
    gt_prediction_association = gt_prediction_association.sort_values(by=['gt_ts', 'id'])
    gt_prediction_association.index = range(len(gt_prediction_association))
    
    # remove NaN rows
    gt_prediction_association = gt_prediction_association.dropna(subset=['id'])
    # idx = 0
    # row_count = len(gt_prediction_association) - 1
    # while idx < row_count:
    #     curr, succ = gt_prediction_association.iloc[idx], gt_prediction_association.iloc[idx + 1]
    #     if curr['gt_ts'] == succ['gt_ts'] and not np.isnan(curr['id']) and np.isnan(succ['id']):
    #         gt_prediction_association = gt_prediction_association.drop([succ.name]) # name = index
    #         row_count -= 1
    #     idx += 1
    
    gt_prediction_association = gt_prediction_association.drop(['keep', 'tracker_ts'], axis=1)
    
    # reorder dataset to fit motchallenge format
    l = len(gt_prediction_association)
    gt_prediction_association = gt_prediction_association[['number', 'id', 'x', 'y']]
    gt_prediction_association = gt_prediction_association.rename(columns={"number": "frame", "x": "bb_left", "y": "bb_top"})
    gt_prediction_association['frame'] = gt_prediction_association['frame'].astype(int)
    gt_prediction_association['id'] = gt_prediction_association['id']
    gt_prediction_association['bb_left'] = gt_prediction_association['bb_left'] - target_width/2
    gt_prediction_association['bb_top'] = gt_prediction_association['bb_top'] + target_width/2
    gt_prediction_association['bb_width'] = [target_width] * l
    gt_prediction_association['bb_height'] = [target_width] * l
    gt_prediction_association['conf'] = [1] * l
    gt_prediction_association['x'] = [-1] * l
    gt_prediction_association['y'] = [-1] * l
    gt_prediction_association['z'] = [-1] * l
    
    return gt_prediction_association

# %%
def main(input_gt_prediction_association:str, input_timestamps_to_keep:str, output_path:str, sample:str, max_missing_frames:int = MAX_MISSING_FRAMES, target_width:int = TARGET_WIDTH):

    args_ok = True
    if not os.path.exists(input_gt_prediction_association):
        print(f"Input timestamps csv file {input_gt_prediction_association} does not exist.")
        args_ok = False

    if not os.path.exists(input_timestamps_to_keep):
        print(f"Input ground truth {input_timestamps_to_keep} does not exist.")
        args_ok = False
    
    # structure MOTChallenge folders
    tracker_data_path = output_path + CHALLENGE_NAME + "-test/tracker/data/"

    if not os.path.exists(tracker_data_path):
        os.makedirs(tracker_data_path)

    if args_ok:
        change_id_prediction = change_track_ids(input_gt_prediction_association, input_timestamps_to_keep, max_missing_frames, target_width)
        change_id_prediction.to_csv("".join([tracker_data_path, sample, ".txt"]), index=False, header=False)
    else:
        print('input_gt_prediction_association', input_gt_prediction_association)
        print('input_timestamps_to_keep', input_timestamps_to_keep)
        raise FileNotFoundError(f"Input csv file does not exist.")

input_gt_prediction_association = BAG_PATH + '/' + sample + '/out/gt_prediction_association.csv'
input_timestamps_to_keep = BAG_PATH + '/' + sample + '/out/timestamps_to_keep.csv'
output_path = BAG_PATH + "/trackers/mot_challenge/"

main(input_gt_prediction_association, input_timestamps_to_keep, output_path, sample)
