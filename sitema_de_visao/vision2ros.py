import rospy
from geometry_msgs.msg import Point # Receiving in this format in kinematics
import vision-lib as vs

def talker():
    pub = rospy.Publisher('position', Point, queue_size=10)
    rospy.init_node('pyarm_vision', anonymous=True)
    rate = rospy.Rate(10) # Rate for activate the loop
    while not rospy.is_shutdown():
        pos = Point()

        # Receiving parameter

        target_color = rospy.get_param('target_color')
        target_direction = rospy.get_param('target_direction')

        # Put your code here

        # Pegar um frame da camera
        masks = vs.load_masks()
        vs.frame = vs.get_frame()
        mask = vs.apply_mask(frame, target_color, masks)

        pos.x, pos,y, pos.z = vs.get_points(mask)

        # Publishing an position
        #pos.x = 10
        #pos.y = 20
        #pos.z = 30

        rospy.loginfo(pos)    # Check
        pub.publish(pos)      # Publish
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

def get_frame():
    '''
        Pega um frame da câmera e retorna para uma variável
    '''

    return (10,20,30)