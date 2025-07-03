# %%
import argparse
import os
from typing import List, Tuple
import pandas as pd

# %% tags=["parameters"]
# declare a list tasks whose products you want to use as inputs
upstream = []
products = None
sample = os.environ.get("INPUT_BAG", "ex01_01")

# %%
TIMESTAMPS_TO_KEEP = '/data/ex08_01/out/timestamps_to_keep.csv'
TRACKER_OUTPUT = '/data/ex08_01/out/tracker_output.csv'
MAX_MISSING_FRAMES = 6
OUTPUT_CSV="/data/out/"
BAG_PATH = os.environ.get("BAG_REPO", "/data")
debug = False

print(upstream)
  
# %%
# given the lists of timestamps gt_ts, searches the closest timestamp to prediction_time
# starting from the timestamp in position start_idx 
def search_closest(prediction_time :int, gt_ts: List[int], start_idx: int) -> Tuple[int, int]:
    idx = start_idx
    diff = abs(prediction_time - gt_ts[idx])
    run = True
    prev_diff = None
    while run and (idx + 1 < len(gt_ts)):
        prev_diff = diff
        idx += 1
        diff = abs(prediction_time - gt_ts[idx])
        if diff > prev_diff:
             run = False

    if (prev_diff is None) or (idx + 1 == len(gt_ts) and prev_diff > diff):
        idx = idx + 1

    return idx - 1
        
def data_alignment(input_tracker_output: str, input_timestamps_to_keep: str):
    
    tracker_output = pd.read_csv(input_tracker_output, sep=';')
    timestamps_to_keep = pd.read_csv(input_timestamps_to_keep)
    tracker_output_ts = tracker_output['timestamp']
    
    # get unique timestamps in tracker output
    tracker_output_ts = tracker_output_ts.groupby(tracker_output_ts).count().index
    gt_ts = timestamps_to_keep['timestamp']

    #gt_ts = [0, 4, 8, 12, 16, 20, 24, 28]
    #tracker_output_ts = [1, 7, 8, 9.1, 9.5, 11, 11.67, 13, 15, 17, 24.1, 26.9, 28.4, 30, 31]
    #tracker_output_ts = [1, 24.1, 26.9, 28.4, 30, 31]

    associations = {}
    idx = 0
    # associates to each timestamp in the tracker output a timestamp in the ground truth
    for time in tracker_output_ts:
        idx = search_closest(time, gt_ts, idx)
        if gt_ts[idx] in associations:
            current_time = associations[gt_ts[idx]]
            # this check is needed if the framerate of the people tracker is higher than
            # that of the ground truth. If more people tracker timestamps match the same 
            # ground truth timestamp, only the closest to the ground truth timestamp is kept
            if abs(time - gt_ts[idx]) < abs(current_time - gt_ts[idx]):
                associations[gt_ts[idx]] = time
        else:
            associations[gt_ts[idx]] = time
        
    
    if debug:
        for key, value in associations.items():
            # test to verify the difference between the associations. They should be
            # no more than half of the time between two consecutive ground truth frames.
            # i.e. 16.6 milliseconds
            print((key - value)/10**6)

    # create a pandas df from the association dictionary
    associations_data = { 'gt_ts': associations.keys(),
                      'tracker_ts': associations.values()}
    
    associations_df = pd.DataFrame(associations_data)

    # create a dataframe with the original track information, plus the associated ground
    # truth timestamp and the information regarding the need to keep or not the timestamp (this
    # information is added with the second merge)
    tmp_df = pd.merge(
        left = tracker_output,
        right = associations_df,
        left_on = 'timestamp',
        right_on = 'tracker_ts',
        how = 'inner'
    )
    tracker_output_with_gt_info = pd.merge(
        left = tmp_df,
        right = timestamps_to_keep,
        left_on = 'gt_ts',
        right_on = 'timestamp',
        how='right'
    )

    tracker_output_with_gt_info['gt_ts'] = tracker_output_with_gt_info['timestamp_y']
    tracker_output_with_gt_info = tracker_output_with_gt_info[['id', 'x', 'y', 'tracker_ts', 'gt_ts', 'number', 'keep']]
    tracker_output_with_gt_info['tracker_ts'] = tracker_output_with_gt_info['tracker_ts'].fillna(-1).astype(int).replace(-1, None)
    tracker_output_with_gt_info['gt_ts'] = tracker_output_with_gt_info['gt_ts'].fillna(-1).astype(int).replace(-1, None)

    return tracker_output_with_gt_info      

# %%
def main(input_timestamps_to_keep:str, input_tracker_output:str, output_path:str):
    args_ok = True
    if not os.path.exists(input_tracker_output):
            print(f"Input tracker output {input_tracker_output} does not exist.")
            args_ok = False
    
    if not os.path.exists(input_timestamps_to_keep):
            print(f"Input ground truth {input_timestamps_to_keep} does not exist.")
            args_ok = False

    if args_ok:
        tracker_output_with_gt_info = data_alignment(input_tracker_output, input_timestamps_to_keep)
        tracker_output_with_gt_info.to_csv("".join([output_path, "gt_prediction_association.csv"]))
    else:
        print('input_timestamps_to_keep', input_timestamps_to_keep)
        print('input_tracker_output', input_tracker_output)
        raise FileNotFoundError(f"Input csv file does not exist.")

input_timestamps_to_keep = BAG_PATH + "/" + sample + "/out/timestamps_to_keep.csv"
input_tracker_output = BAG_PATH + "/" + sample + "/tracker_output.csv"
output_path = BAG_PATH + "/" + sample + "/out/"

main(input_timestamps_to_keep, input_tracker_output, output_path)
