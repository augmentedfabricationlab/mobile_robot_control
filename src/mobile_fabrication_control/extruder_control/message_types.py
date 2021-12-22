
__all__ = [
    "MSG_RECEIVED",
    "MSG_EXECUTED",
    "MSG_STOP",
    "MSG_INFO",
    "MSG_DODATA",
    "MSG_MOTORDATA",
    "MSG_MOTORSTATE",
    "msg_types_dict"
]

MSG_RECEIVED = 0
MSG_EXECUTED = 1
MSG_STOP = 2
MSG_INFO = 3
MSG_DODATA = 4
MSG_MOTORDATA = 5
MSG_MOTORSTATE = 6

# key : (name, send, recv)
msg_types_dict = {
    0: ("MSG_RECEIVED", "", ""),
    1: ("MSG_EXECUTED", "", ""),
    2: ("MSG_STOP", "", ""),
    3: ("MSG_INFO", "", ">16s16s16sl"),
    4: ("MSG_DODATA", "ll", ""),
    5: ("MSG_MOTORDATA", "lff", ""),
    6: ("MSG_MOTORSTATE", "l", "")
}
