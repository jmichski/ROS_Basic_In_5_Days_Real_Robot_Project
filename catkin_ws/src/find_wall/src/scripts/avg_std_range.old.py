def average_std_range(ranges, index, range_min, window=SCAN_AVERAGE_WINDOW):
    """Average and std dev of 'window' samples on each side of index.
    inf values replaced with range_max for calculation.
    Returns (inf, inf) if 4 or more samples are inf."""
    samples   = []
    inf_count = 0
    for i in range(index - window, index + window + 1):
        val = ranges[i % len(ranges)]
        if math.isinf(val):
            inf_count += 1
            samples.append(range_min)   # replace inf with range_max
        elif not math.isnan(val):
            samples.append(val)
    if inf_count >= 4:
        return float('inf'), float('inf')
    if not samples:
        return float('inf'), float('inf')
    mean    = sum(samples) / len(samples)
    std_dev = math.sqrt(sum((x - mean) ** 2 for x in samples) / len(samples))
    return mean, std_dev