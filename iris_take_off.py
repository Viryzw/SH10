from VENI.proxy import UAVController
import rospy
import VENI.trajectory as trajectory

if __name__ == "__main__":

    iris = UAVController("iris", "0", takeOffOffset=[2.5, 2.7, 0.5])
    flag = 0
    rate = rospy.Rate(20)
    for i in range(100):
        iris.goto_position(0, 0, 5.0)
        iris.send_command("OFFBOARD")
        rate.sleep()
        iris.send_command("ARM")
    while not rospy.is_shutdown():
        print(iris.state.mode)
        if flag == 0:
            reach1 = iris.goto_position(0, 0, 5.0)
            if reach1 :
                flag = 1
                points = trajectory.plan_traj([0,0], iris.gps_point, iris.t1, iris.t2, 250)
        if flag == 1:
            p1, p2 = trajectory.extend_point(iris.X, iris.Y, points[pIndex][0], points[pIndex][1], 30)            
            print(points[pIndex])
            print(pIndex)
            iris.goto_position(points[pIndex][0], points[pIndex][1], 25)
            if iris._is_arrived(points[pIndex][0], points[pIndex][1], 25, threshold=30) and pIndex < len(points) - 1:
                pIndex = pIndex + 1
            if False:  # detect
                flag = 2
                count = 0
        
        if flag == 2:
            iris.goto_position(0,0,25)
            if iris._is_arrived(0,0,25, threshold=2):
                count += 1
            else:
                count = 0

            if count > 80:
                iris.send_command("AUTO.LAND")
        rate.sleep()


