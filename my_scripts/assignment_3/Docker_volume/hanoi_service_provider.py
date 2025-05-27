# filepath: /root/catkin_ws/src/my_scripts/scripts/hanoi_service_provider.py
#!/usr/bin/env python

import rospy
from my_scripts.srv import TowerMove, TowerMoveResponse # Corrected import

def handle_execute_hanoi_move(req):
    """
    Handles the service request to execute a Tower of Hanoi move.
    For now, it just logs the move and returns success.
    """
    rospy.loginfo(f"Service Request: Move disk {req.disk} from {req.source_peg} to {req.destination_peg}")
    
    # In a real scenario, you would add logic here to:
    # 1. Validate the move.
    # 2. Update the state of the Tower of Hanoi game.
    # 3. Interact with a robot or simulation to perform the move.
    
    # For this example, we'll assume the move is always successful.
    response_message = f"Successfully processed move: Disk {req.disk} from {req.source_peg} to {req.destination_peg}"
    rospy.loginfo(response_message)
    return TowerMoveResponse(success=True, message=response_message)

def hanoi_move_service_provider():
    rospy.init_node('hanoi_move_service_provider_node')
    s = rospy.Service('/execute_hanoi_move', TowerMove, handle_execute_hanoi_move)
    rospy.loginfo("Hanoi move service provider ready.")
    rospy.spin()

if __name__ == "__main__":
    try:
        hanoi_move_service_provider()
    except rospy.ROSInterruptException:
        pass