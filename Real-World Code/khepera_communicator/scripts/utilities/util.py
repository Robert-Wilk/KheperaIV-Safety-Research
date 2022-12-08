def filter_ip(arr):
    for i in range(len(arr)):
        arr[i] = arr[i].split('_')[0]
    return arr
