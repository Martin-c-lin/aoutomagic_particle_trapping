# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 08:49:16 2020

@author: Martin Selin
"""

# TODO: use_LGO (and LGO_order?) are lists of bools not numbers.
float_parameters = ['LGO_order', 'setpoint_temperature',
                    'recording_duration', 'target_experiment_z',
                    'SLM_iterations']

bool_parameters = ['temperature_output_on', 'activate_traps_one_by_one',
                   'need_T_stable']
bool_list = ['use_LGO']
float_list = ['xm', 'ym']


def Line2KeyValue(line):
    '''

    Function for converting a string(line in file) to a key-value pair.
    Parameters
    ----------
    line : TYPE - String ordered as 'key':'value'
        DESCRIPTION. String to be converted to key, value pair of dictionary

    Returns
    -------
    TYPE String
        Key of line in dicionary
    TYPE - depends on the key. Either float, list or bool
        Value corresponding to the key.

    '''
    key, string_value = None, None

    # Try to split the line into a key and value pair
    idx_colon = line.find(':')
    if idx_colon > 0:
        key = line[:idx_colon]
        string_value = line[idx_colon+1:]
    else:
        return key, string_value

    # Check if the key is an integer in which case this means that it is a new
    # dict required.
    try:
        dict_idx = int(key)
        return dict_idx, string_value
    except:
        pass

    # Convert the value from string to, either float, list or bool.
    try:
        if key in float_parameters:
            return key, float(string_value)
        elif key in bool_parameters:
            value = (string_value == 'True')
            return key, value
        elif key in float_list:
            value_list = [float(s) for s in string_value.split(',')]
            return key, value_list
        elif key in bool_list:
            value_list = [(s=='True') for s in string_value.split(',')]
            return key, value_list
    except:
        print('Warning, could not convert value in ', string_value)
        return None, None

    # Key not in any of the lists.
    print('Error, key not found')
    return None, None


def ReadFileToExperimentList(filepath):
    '''
    Function for reading a .txt file into a list of dicts describing an
    experiment schedule.

    Parameters
    ----------
    filepath : String
        Path to the file which is to be read.

    Returns
    -------
    dict_list : List of dicts.
        List of dictionaries containing experiment setups read from the file.

    '''
    dict_list = []
    current_dict = {}
    # Open the file for reading.
    try:
        file_reader = open(filepath, 'r')
    except FileNotFoundError:
        print('Could not find the file you were looking for')
        return None
    data = file_reader.readlines()  # Read all the lines in the file to a list.
    file_reader.close()

    # Look through all the lines of code
    for idx, line in enumerate(data):
        key, value = Line2KeyValue(line)
        # If the key is an integer then we need a new dict.
        if isinstance(key, int) and len(current_dict) > 0:
            dict_list.append(current_dict)
            current_dict = {}
        elif key is not None:
            current_dict[key] = value
        else:
            print('Could not read line', idx+1, ' into anything meaningfull.')

    # Add last dictionary if it has not yet been added,
    if len(current_dict) > 0:
        dict_list.append(current_dict)

    return dict_list
