while(True):
    try:
        string = input()
        for i in range(4):
            print(string[32 - 8 * (i + 1):32 - 8 * (i) ])

    except Exception:
        break