# Wire delay

1 edge 1 delay

# Buffered-DME (Deferred-Merge Embedding)

partition -> topology generation -> DME bottom-up -> DME top-down allocation -> bottom-up route -> detour repair -> buffer insertion

# 优先级

合法可布线性 > skew 潜力 > wirelength > buffer cost

## 原因

```python
score = 5000000 − 5000 * skew−50 * wirelength − 200 * buﬀercost
```

结论： `wirelength` 是最该被浪费的东西，修 skew 优先使用 detour。

# buffer insertion

top-down顺序
在每一层所有 ms node上 枚举每一种 buffer:
    count down-stream sinks number
    如果 down-stream sinks number > buffer fan-out
        pass
    选一个对 skew 最好的位置以及对应的 buffer 种类插入

