import rospy 
from std_msgs.msg import String


rospy.init_node('aula4_1')


mensagem = ''

def recebe(mesagem_recebida):
    global mensagem
    mensagem = mesagem_recebida.data


def timerCallBack(event):
    print(mensagem)
    msg = String()
    msg.data = '2017006353'
    pub.publish(msg)
    

pub = rospy.Publisher('/topic1', String, queue_size=1)
timer = rospy.Timer(rospy.Duration(0.1), timerCallBack)
sub = rospy.Subscriber('/topic2', String, recebe)

rospy.spin()