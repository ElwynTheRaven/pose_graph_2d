# 读取 result_init.csv 和 result_opt.csv（列：id, x, y, theta），保存为 compare.png
import csv
 
import matplotlib
matplotlib.use("Agg")  # 不弹窗口，直接保存图片
import matplotlib.pyplot as plt
 
 
def load(path):
    xs, ys = [], []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)  # 跳过表头
        for r in reader:
            if r:
                xs.append(float(r[1]))
                ys.append(float(r[2]))
    return xs, ys
 
 
before = load("result_init.csv")
after = load("result_opt.csv")
 
fig, axes = plt.subplots(1, 2, figsize=(13, 6.5))
for ax, (xs, ys), title in zip(axes, [before, after], ["Before optimization", "After optimization"]):
    ax.plot(xs, ys, linewidth=0.8, color="#534AB7")
    ax.plot(xs[0], ys[0], "o", color="#D85A30", label="start")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend()
fig.tight_layout()
fig.savefig("compare.png", dpi=150)
print("saved compare.png")