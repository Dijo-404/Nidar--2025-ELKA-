# This is a simple script to switch between 0 and 1 to check if the camera is getting triggered or not
# This code uses relative path for better modularity 
# Hope this code gets ur approval bros :)
# not using os because we already create an text file and mention its path within open function 
# We could use a while True loop to keep the program running and use a keyboard interrupt to stop if u want that let me know 

import argparse
def switch_code(trigger):
    s = [0,1]  
    with open("file path ", "w") as txt_file:
        txt_file.write(str(s[trigger])) 
#-------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Choose which number to switch [0 or 1]')
    parser.add_argument(
        'switch',
        type=int,
        choices=[0, 1],
        help='Choose which number to switch [0 or 1]'
    )
    args = parser.parse_args()
    switch_code(args.switch)
#-------------------------------------------------------------------------------------------------#