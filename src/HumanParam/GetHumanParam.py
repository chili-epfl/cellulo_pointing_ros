import numpy as np
import json

def main():
    '''
    Based on keypoints extracted with pose estimations framework, save human morphological parameters in mm
    '''
    
    # Open file containing the key points obtained from the pose estimation library
    with open('/home/lburget/Documents/EPFL/Master/PDS2/Lucas/HumanParam/2_keypoints.json') as json_file:  
        data = json.load(json_file)

    poseH = np.array(data["people"][0]["pose_keypoints_2d"]).reshape(25,3)
    hand = np.array(data["people"][0]["hand_right_keypoints_2d"]).reshape(21,3)

    ratio = 190/dist(hand[0], hand[12]) # size of the hand is measured and used to know the ratio pixels/true size
    print(ratio)

    humanParam = dict()

    # Armlength
    humanParam["armlength"] = ratio*(dist(poseH[6], poseH[7]) \
                +  dist(poseH[5], poseH[6]) \
                + dist(hand[0], hand[12]))

    # Human parameter in mm

    humanParam["height_eye"] = 1640 #(poseH[16][1]+poseH[15][1])/2 - (poseH[11][1]+poseH[14][1])/2\
    # if poseH[14][0] != 0 else 0  
    humanParam["center_eye"] = np.abs(poseH[15][0]-poseH[0][0])*ratio
    humanParam["eyeshouldervector_y"] = np.abs(poseH[2][0]-poseH[1][0])*ratio
    humanParam["eyeshouldervector_z"] = np.abs(poseH[15][1]-poseH[1][1])*ratio
    print(humanParam)

    with open("/home/lburget/Documents/EPFL/Master/PDS2/Lucas/HumanParam/LUCASV23.json", 'w+') as file_write:
        json.dump(humanParam, file_write)

def dist(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1] -b[1])**2)

if __name__ == "__main__":
    main()