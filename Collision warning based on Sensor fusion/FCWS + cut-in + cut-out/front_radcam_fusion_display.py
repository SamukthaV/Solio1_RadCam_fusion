import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32MultiArray, String
from conti_radar.msg import radar_obj
from datetime import datetime
import numpy as np

rospy.init_node('listener', anonymous=True)

a = []  # List to store radar data timestamps
b = []  # List to store camera data timestamps
i = 0   # Flag to indicate the presence of camera data
current_x = None
current_y = None  # Variable to store the current radar data
range = None
RCS = None
SNR = None
current_bounding_boxes = None  # Variable to store the current bounding boxes

# Load camera projection matrix
ndlt = np.load('/home/orin/Downloads/NDLT_matrix_solio_new.npy')

# Radius of the circles to be drawn
circle_radius = 4

# Color in (B, G, R) format for drawing circles
colors = (0, 0, 255)

def compute_world2img_projection(world_points, M, is_homogeneous=False):
    if not is_homogeneous:
        points_h = np.vstack((world_points[:3, :], np.ones(world_points.shape[1])))

    h_points_i = M @ points_h

    h_points_i[0, :] = h_points_i[0, :] / h_points_i[2, :]
    h_points_i[1, :] = h_points_i[1, :] / h_points_i[2, :]

    points_i = h_points_i[:2, :]

    return points_i

def callback(data):
    timestamp = datetime.now().strftime('%H:%M:%S.%f')
    # print(timestamp)
    a.append(timestamp)
    global current_x, current_y, vabsx, vabsy, arelx, arely, range
    current_x, current_y, vabsx, vabsy = data.f_DistX, data.f_DistY, data.f_VabsX, data.f_VabsY

def time_callback(msg):
    global b, i
    # rospy.loginfo("Received Timestamp for Image %s", msg.data)
    b.append(msg.data)

def unflatten_data(flattened_data):
    list_of_lists = []
    sublist = []
    for item in flattened_data:
        if item == -1:
            list_of_lists.append(sublist)
            sublist = []
        else:
            sublist.append(item)
    return list_of_lists

def bbox_callback(msg):
    global current_bounding_boxes
    list_of_lists = unflatten_data(msg.data)
    # rospy.loginfo("################Received data of bbox_callback: %s", list_of_lists)
    current_bounding_boxes = list_of_lists

def put_text_center(class_name,img, text, font_scale=1, color=(0, 0, 255), thickness=2):

    # Get the size of the text
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]

    # Calculate the x position to center the text
    text_x = int((img.shape[1] - text_size[0]) / 2)

    # Set the y position to be just slightly below the top of the image
    if (class_name!=0.0):
        text_y = text_size[1] + 10  # 10 pixels padding from the top
    else:
        text_y = text_size[1] + 50
    # Put the text on the image
    cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)


def put_text_top_left_of_box(img, text, box_coords, font_scale=2.5, color=(0, 0, 255), thickness=3):

    xmax, xmin, ymax, ymin = box_coords

    # Calculate the top-left corner of the bounding box
    x = xmin
    y = ymin

    # Get the size of the text
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_PLAIN, font_scale, thickness)[0]

    # Set the position for the text (top-left of the bounding box, slightly above it)
    text_x = x
    text_y = y - 10  # 10 pixels above the top of the bounding box

    # Adjust y position if the text goes out of the image frame
    if text_y < 0:
        text_y = y + text_size[1] + 10

    # Put the text on the image
    cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_PLAIN, font_scale, color, thickness)

def put_text_bottom_left_of_box(img, text, box_coords, font_scale=2.5, color=(0, 0, 255), thickness=3):
    """
    Places the specified text on the bottom-left corner outside the bounding box.
    
    Parameters:
        img (numpy.ndarray): The image on which to place the text.
        text (str): The text string to display.
        box_coords (tuple): The bounding box coordinates in the format (xmax, xmin, ymax, ymin).
        font_scale (float): The scale of the text.
        color (tuple): The color of the text in BGR format.
        thickness (int): The thickness of the text.
    """
    xmax, xmin, ymax, ymin = box_coords

    # Calculate the bottom-left corner of the bounding box
    x = xmin
    y = ymax

    # Get the size of the text
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_PLAIN, font_scale, thickness)[0]

    # Set the position for the text (bottom-left of the bounding box, slightly below it)
    text_x = x
    text_y = y + text_size[1] + 10  # 10 pixels below the bottom of the bounding box

    # Adjust y position if the text goes out of the image frame
    if text_y > img.shape[0]:  # If the text goes below the image
        text_y = y - 10  # Place it inside the bounding box instead

    # Put the text on the image
    cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_PLAIN, font_scale, color, thickness)

def image_callback(msg):
    global cv_image, current_bounding_boxes, current_x, current_y, range

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    global check
    check = 0
    vehicle_cmd_pub = rospy.Publisher('/fcws_commands', String, queue_size=10)
    if cv_image is not None and current_y is not None:
        # Extract relevant radar data
        x = np.asarray(current_x)
        y = -np.asarray(current_y)
        range = np.sqrt(np.square(x) + np.square(y))
        velp = np.asarray(vabsx)
        velq = np.asarray(vabsy)
        vel = np.sqrt(np.square(velp) + np.square(velq))

        # Project radar points onto the image
        predictions = compute_world2img_projection(np.vstack((x, y, np.ones_like(x) * 0.78)), ndlt)
        predictions = np.round(predictions.T)

        for class_name, x_min, y_min, x_max, y_max in current_bounding_boxes:
            if class_name == 0 or class_name == 1 or class_name ==2:
                x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)

                # Find the radar point with the shortest range within the bounding box
                min_range_index = None
                min_range = float('inf')
                for i, (px, py) in enumerate(predictions):
                    px, py = int(px), int(py)
                    if x_min <= px <= x_max and y_min <= py <= y_max:
                        if range[i] < min_range and -2.0 <= current_y[i] <= 2.0:   #road width
                            min_range = range[i]
                            min_range_index = i
                

                # If a radar point is found within the bounding box, display it
                if min_range_index is not None:
                    px, py = predictions[min_range_index]
                    r = range[min_range_index]
                    r = r - 4.0
                    v = vel[min_range_index]
                    print("#########################",class_name)
                    # Print information about the closest radar point
                    print(f"Bounding Box: {class_name} ({x_min}, {y_min}) - ({x_max}, {y_max})")
                    print(f"Closest Point: ({px}, {py}), Range: {r:.2f} m, Velocity: {v:.1f} m/s")
                    v= v*3.6 
                    print(f"Velocity: {float(v)} kmph")
                    # Draw the bounding box on the image
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                    # Draw a circle and text for the closest radar point
                    cv2.circle(cv_image, (int(px), int(py)), circle_radius, colors, -1)
                    text = f"Range: {r:.2f} m"
                    v = round(v,2)
                    velocity = f"Velocity: {float(v)} kmph"
                    print("################################33 class name",class_name)
                    put_text_top_left_of_box(cv_image, text, [x_max,x_min,y_max,y_min], font_scale=1.5, color=(235, 206, 135), thickness=2)
                    # cv2.putText(cv_image, text, (int(px), int(py) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0., colors, 2)
                    put_text_bottom_left_of_box(cv_image, velocity, [x_max,x_min,y_max,y_min], font_scale=1.5, color=(235, 206, 135), thickness=2)
                    # cv2.putText(cv_image, velocity, (int(px), int(py) +40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors, 2)
                    if class_name == 0.0:
                        out = "PEDESTRIAN AHEAD"
                    elif class_name == 1.0:
                        out = "BICYCLE AHEAD"
                    elif class_name == 2.0:
                        out = "CAR AHEAD"
                    elif class_name == 7.0:
                        out = "TRUCK AHEAD"
                    else:
                        out = "Unknown Object Ahead"

                    # Apply the condition for y value
                    if 0 <= r <= 10:
                        text = out + " STOP"
                        vehicle_cmd_pub.publish("STOP")
                        put_text_center(class_name,cv_image, text, 1, (0, 0, 255), 2)
                        check = 1
                    elif 10 < r <= 20:
                        text = out + " SLOW DOWN"
                        vehicle_cmd_pub.publish("SLOW")
                        put_text_center(class_name,cv_image, text, 1, (0, 0, 255), 2)
                        check = 2
                    else:
                        text = out + " GO"
                        vehicle_cmd_pub.publish("GO")
                        put_text_center(class_name,cv_image, text, 1, (0, 255, 0), 2)

            if check == 0:
                vehicle_cmd_pub.publish("GO")
            # Show the modified image
            cv2.imshow("Sensor Fusion", cv_image)

            cv2.imwrite('/home/orin/basler_v8/Collision warning based on Sensor fusion/FCWS + cut-in + cut-out/image/', cv_image)
            cv2.waitKey(1)



rospy.Subscriber("/radar_lrr_front_obj", radar_obj, callback)
rospy.Subscriber("/object_topic_front", Float32MultiArray, bbox_callback)
rospy.Subscriber("/time_topic_front", String, time_callback)
rospy.Subscriber("/basler_front", Image, image_callback)

rospy.spin()
