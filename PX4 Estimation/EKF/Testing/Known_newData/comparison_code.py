import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from tabulate import tabulate

def compare_csv_files(testing_file_path, original_file_path):
    """
    Compare two CSV files with orientation data and visualize differences.
    
    Parameters:
    testing_file_path (str): Path to the testing output CSV
    original_file_path (str): Path to the original filtered output CSV
    """
    # Read both CSV files
    testing_df = pd.read_csv(testing_file_path)
    original_df = pd.read_csv(original_file_path)
    
    # Print basic information about both files
    print("\n--- File Information ---")
    print(f"Testing file '{testing_file_path}':")
    print(f"  Rows: {testing_df.shape[0]}")
    print(f"  Columns: {', '.join(testing_df.columns)}")
    
    print(f"\nOriginal file '{original_file_path}':")
    print(f"  Rows: {original_df.shape[0]}")
    print(f"  Columns: {', '.join(original_df.columns)}")
    
    # Find common columns between the two dataframes
    common_cols = [col for col in original_df.columns if col in testing_df.columns]
    print(f"\nCommon columns: {', '.join(common_cols)}")
    
    # Calculate differences for each common column
    diff_stats = {}
    for col in common_cols:
        diffs = np.abs(testing_df[col].values - original_df[col].values)
        mean_diff = np.mean(diffs)
        max_diff = np.max(diffs)
        min_diff = np.min(diffs)
        
        # Calculate percentage difference based on mean of original values
        mean_original = np.mean(np.abs(original_df[col].values))
        percent_diff = (mean_diff / mean_original) * 100 if mean_original != 0 else float('inf')
        
        diff_stats[col] = {
            'mean_diff': mean_diff,
            'max_diff': max_diff,
            'min_diff': min_diff,
            'percent_diff': percent_diff
        }
    
    # Display difference statistics
    print("\n--- Difference Statistics ---")
    stats_table = []
    for col, stats in diff_stats.items():
        stats_table.append([
            col, 
            f"{stats['mean_diff']:.6f}", 
            f"{stats['max_diff']:.6f}", 
            f"{stats['min_diff']:.6f}", 
            f"{stats['percent_diff']:.2f}%"
        ])
    
    print(tabulate(
        stats_table, 
        headers=["Column", "Mean Difference", "Max Difference", "Min Difference", "% Difference"],
        tablefmt="grid"
    ))
    
    # Create merged dataframe for easier comparison
    merged_df = pd.DataFrame()
    for col in common_cols:
        merged_df[f'testing_{col}'] = testing_df[col]
        merged_df[f'original_{col}'] = original_df[col]
        merged_df[f'diff_{col}'] = np.abs(testing_df[col] - original_df[col])
    
    # Display sample data (first 5 rows)
    print("\n--- Sample Data Comparison ---")
    for col in common_cols:
        print(f"\nColumn: {col}")
        sample_table = []
        for i in range(min(5, len(merged_df))):
            sample_table.append([
                i,
                f"{merged_df[f'testing_{col}'].iloc[i]:.6f}",
                f"{merged_df[f'original_{col}'].iloc[i]:.6f}",
                f"{merged_df[f'diff_{col}'].iloc[i]:.6f}"
            ])
            
        print(tabulate(
            sample_table,
            headers=["Row", f"Testing {col}", f"Original {col}", "Difference"],
            tablefmt="grid"
        ))
    
    # Create visualizations for each column
    create_visualizations(merged_df, common_cols)
    
    return diff_stats, merged_df

def create_visualizations(merged_df, columns):
    """
    Create visualizations for the compared data.
    
    Parameters:
    merged_df (DataFrame): DataFrame with testing, original, and diff values
    columns (list): List of common columns to visualize
    """
    for col in columns:
        plt.figure(figsize=(14, 10))
        
        # Plot 1: Values comparison
        plt.subplot(2, 1, 1)
        plt.plot(merged_df.index, merged_df[f'testing_{col}'], 'b-o', label=f'Testing {col}')
        plt.plot(merged_df.index, merged_df[f'original_{col}'], 'g-o', label=f'Original {col}')
        plt.title(f'{col} Values Comparison')
        plt.xlabel('Row Index')
        plt.ylabel('Value')
        plt.grid(True)
        plt.legend()
        
        # Plot 2: Difference
        plt.subplot(2, 1, 2)
        plt.plot(merged_df.index, merged_df[f'diff_{col}'], 'r-o')
        plt.title(f'Absolute Difference in {col}')
        plt.xlabel('Row Index')
        plt.ylabel('Difference')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(f'{col}_comparison.png')
        print(f"Saved visualization for {col} to {col}_comparison.png")
    
    # Create a heatmap of all differences
    plt.figure(figsize=(12, 8))
    diff_cols = [f'diff_{col}' for col in columns]
    diff_df = merged_df[diff_cols].copy()
    diff_df.columns = columns  # Simplify column names for the heatmap
    
    # Create heatmap
    sns.heatmap(diff_df.T, cmap='YlOrRd', annot=True, fmt='.2f')
    plt.title('Differences Heatmap (Rows = Variables, Columns = Data Points)')
    plt.savefig('differences_heatmap.png')
    print("Saved differences heatmap to differences_heatmap.png")

if __name__ == "__main__":
    # Paths to your CSV files
    testing_file = "filtered_output.csv"
    original_file = "original.csv"
    
    # Run the comparison
    stats, merged_data = compare_csv_files(testing_file, original_file)
    
    print("\nAnalysis complete. Visualization files have been saved to the current directory.")
