import os
import pandas as pd
from collections import defaultdict

def analyze_folder(folder_path):
    # Initialize a dictionary to store counts for each model
    model_counts = {
        'transe': 0,
        'simple': 0,
        'rotate': 0,
        'quate': 0,
        'distmult': 0,
        'complex': 0
    }

    # Initialize a dictionary to store hits@3 and hits@5 for each model
    hits_at_3 = defaultdict(lambda: {
        'transe': 0,
        'simple': 0,
        'rotate': 0,
        'quate': 0,
        'distmult': 0,
        'complex': 0
    })
    
    hits_at_5 = defaultdict(lambda: {
        'transe': 0,
        'simple': 0,
        'rotate': 0,
        'quate': 0,
        'distmult': 0,
        'complex': 0
    })

    # Initialize a dictionary to store counts for each groundtruthID per model
    groundtruth_stats = defaultdict(lambda: {
        'transe': 0,
        'simple': 0,
        'rotate': 0,
        'quate': 0,
        'distmult': 0,
        'complex': 0
    })
    
    # Iterate over each file in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".csv"):
            labelID, groundtruthID, _ = filename.split('_')
            
            # Read the CSV file
            file_path = os.path.join(folder_path, filename)
            df = pd.read_csv(file_path)
            
            # Filter out rows where WikiDataID matches labelID
            df_filtered = df[df['WikiDataID'] != labelID]
            
            # Convert relevant columns to numeric, coerce errors to NaN
            for model in ['transe', 'simple', 'rotate', 'quate', 'distmult', 'complex']:
                df_filtered[model] = pd.to_numeric(df_filtered[model], errors='coerce')
            
            # Fill NaN values with a very low number to ensure they don't interfere
            df_filtered = df_filtered.fillna(-float('inf'))
            
            # Iterate over each model column and check the highest value
            for model in ['transe', 'simple', 'rotate', 'quate', 'distmult', 'complex']:
                # Get the row with the maximum value for the current model
                max_row = df_filtered.loc[df_filtered[model].idxmax()]
                
                # Check if the WikiDataID of this row matches the groundtruthID
                if max_row['WikiDataID'] == groundtruthID:
                    model_counts[model] += 1
                    groundtruth_stats[groundtruthID][model] += 1
                
                # Calculate hits@3 and hits@5
                top_3_rows = df_filtered.nlargest(3, model)
                top_5_rows = df_filtered.nlargest(5, model)
                
                if groundtruthID in top_3_rows['WikiDataID'].values:
                    hits_at_3[groundtruthID][model] += 1
                
                if groundtruthID in top_5_rows['WikiDataID'].values:
                    hits_at_5[groundtruthID][model] += 1
    
    # Convert the model_counts dictionary to a DataFrame
    summary_df = pd.DataFrame.from_dict(model_counts, orient='index', columns=['count'])
    summary_df.to_csv('model_comparison_summary.csv')

    # Convert the groundtruth_stats dictionary to a DataFrame
    groundtruth_df = pd.DataFrame.from_dict(groundtruth_stats, orient='index')
    groundtruth_df.to_csv('groundtruth_model_stats.csv')

    # Convert hits_at_3 and hits_at_5 to DataFrames
    hits_at_3_df = pd.DataFrame.from_dict(hits_at_3, orient='index')
    hits_at_5_df = pd.DataFrame.from_dict(hits_at_5, orient='index')

    # Save hits@3 and hits@5 data to CSV files
    hits_at_3_df.to_csv('hits_at_3_summary.csv')
    hits_at_5_df.to_csv('hits_at_5_summary.csv')

# Set the path to the folder containing the CSV files
folder_path = '/home/user/pel_ws/src/pel_ros/results'

# Run the analysis
analyze_folder(folder_path)
