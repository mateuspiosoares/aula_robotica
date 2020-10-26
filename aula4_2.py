import rospy 
from std_msgs.msg import String


rospy.init_node('aula4_2')

mensagem_recebida = '0'

def timerCallBack(event):
    global mensagem_recebida
    mensagem_recebida = event.data
    soma = sum(int(i) for i in mensagem_recebida)
    event.data = str(soma)
    pub.publish(event)
    

pub = rospy.Publisher('/topic2', String, queue_size=1)
sub = rospy.Subscriber('/topic1', String, timerCallBack)

rospy.spin()