switcher = {
    "0000" : "0",
    "0001" : "1",
    "0010" : "2",
    "0011" : "3",
    "0100" : "4",
    "0101" : "5",
    "0110" : "6",
    "0111" : "7",
    "1000" : "8",
    "1001" : "9",
    "1010" : "a",
    "1011" : "b",
    "1100" : "c",
    "1101" : "d",
    "1110" : "e",
    "1111" : "f"
}

# while(True):
#     try:
        
#         string = input()
#         for i in range(8):
#             ali_khar = ali_khar + switcher[string[0 + i * 4 :4 + i * 4]]
#         print(ali_khar)
#     except Exception:
#         break

from collections import deque
insts = deque()
while(True):
    try:
        string = input()
        insts.appendleft(string)
        
    except Exception:
        break
    
for x in insts:
    splitted_inst = x.split()
    ali_khar = ""
    for f in range(4):
        for i in range(2):
            ali_khar = ali_khar + switcher[splitted_inst[f][0 + i * 4 :4 + i * 4]]
    ali_khar = ali_khar + " ".join(splitted_inst[4:])
    print(ali_khar)