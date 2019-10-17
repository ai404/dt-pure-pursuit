import pandas as pd
import matplotlib.pyplot as plt

files = ["logs-control2", "logs-errors2"]
for file in files:
    plt.figure(figsize=(10,5))
    errors_df = pd.read_csv("%s.csv"%file)
    column1, column2 = errors_df.columns[1:]
    errors_df["Time"] -= errors_df["Time"].min()
    plt.plot(errors_df["Time"], errors_df[column1], label=column1)
    plt.plot(errors_df["Time"], errors_df[column2], label=column2)
    plt.legend()
    plt.savefig("%s-%s.jpg"%(column1,column2))