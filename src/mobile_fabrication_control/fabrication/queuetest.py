from queue import Queue
import json

item = {"LAYER":0, "NODE":0}
print(str(item))
stritem = "{'LAYER':0, 'NODE':0}"
# node_msg = json.loads(stritem)
q = Queue()
q.put(str(item))
getitem = q.get()
print(getitem)
print(type(getitem))

contents = getitem[1:-1].replace("'", '"')
print(contents)