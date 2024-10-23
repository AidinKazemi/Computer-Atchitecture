from collections import deque
insts = deque()
while(True):
    try:
        string = input()
        insts.appendleft(string)
        # for i in range(4):
            # print(string[32 - 8 * (i + 1):32 - 8 * (i) ])

    except Exception:
        break
for x in insts:
    splitted_inst = x.split()
    for i in reversed(range(4)):
        print(splitted_inst[i])