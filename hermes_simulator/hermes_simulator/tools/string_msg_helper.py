from std_msgs.msg import String
import json

def create_string_msg_from(dict):
    '''
    Creates a string message from the given dictionary.

    Parameters:
    - dict(Dict): the contents of the message as a dictionary.

    Returns:
    - result_msg(String): the message as a String message.
    '''
    result_msg = String()
    result_msg.data = json.dumps(dict)
    return result_msg

def get_msg_content_as_dict(msg):
    '''
    Converts the contents of the String message to a dictionary.

    Parameters:
    - msg(String): the string message.

    Returns:
    - (Dict): the contents of the message as a dictionary.
    '''
    return json.loads(msg.data)