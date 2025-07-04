import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import os

def main():
    log_dir = Path.home() /"qr_nav_ws"/"logs"
    csv_files = sorted(log_dir.glob("*.csv"), key=os.path.getmtime, reverse=True)

    if not csv_files:
        print("No CSV files have been found in ~/qr_nav_ws/logs")
        return

    latest_csv = csv_files[0]
    print(f"Loading: {latest_csv}")

    df = pd.read_csv(latest_csv)
    df.columns = df.columns.str.strip()

    if not {'pozitie_x', 'pozitie_y', 'pozitie_z', 'id'}.issubset(df.columns):
        print("The CSV file does not have the necessary columns.")
        return

    unique_tags = df['id'].unique()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for tag in unique_tags:
        tag_data = df[df['id'] == tag]
        ax.scatter(tag_data['pozitie_x'], tag_data['pozitie_y'], tag_data['pozitie_z'], label=str(tag))

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Position of detected codes')
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

