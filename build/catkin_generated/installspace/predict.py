#!/usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from sklearn.linear_model import Ridge, ElasticNet
from hmmlearn.hmm import GaussianHMM
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

hist_traj = "onboard_detector/history_trajectories"
class traj_predict:
    def __init__(self):
        self.hist_traj_received = False
        self.hist_sub = rospy.Subscriber(hist_traj, MarkerArray, self.hist_traj_callback)
        
        # self.hist_pub = rospy.Publisher("traj_predict/hist_traj", MarkerArray, queue_size=10)
        self.pred_pub = rospy.Publisher("traj_predict/pred_traj", MarkerArray, queue_size=10)
        rospy.Timer(rospy.Duration(0.033), self.pred_callback)
        
    def hist_traj_callback(self, msg):
        self.hist_traj = msg
        self.hist_traj_received = True
        
    def pred_callback(self, event):
        if (self.hist_traj_received == True):
            self.pred()
            self.pred_pub.publish(self.pred_traj)
            # print("1")
            
    def pred(self):
        self.pred_traj = MarkerArray()
        numob = 0
        for traj in self.hist_traj.markers:
            numob += 1
            y_trainx = []
            y_trainy = []
            z = []
            X_train = []
            X_test = []
            time = 0
            for p in traj.points:
                y_trainx.append(p.x)
                y_trainy.append(p.y)
                z.append(p.z)
                # X_train.append([1, time, time**2, time**3, time**4])
                # X_train.append([p.x, p.y])
                X_train.append(time)
                time += 1
            y_trainx = np.array(y_trainx)
            y_trainy = np.array(y_trainy)
            X_train = np.array(X_train)
            # X_train.reshape(-1,1)
            # y_trainx.reshape(-1,1)
            # y_trainy.reshape(-1,1)
            
            
            for i in range(30):
                # X_test.append([1, time, time**2, time**3, time**4])
                X_test.append(time)
                time += 1 
            X_test = np.array(X_test)
            # X_test.reshape(1,-1)
            
            # kernel = C(1.0, (1e-4, 1e1)) * RBF(1, (1e-4, 1e1))
            # regressorx = Ridge(alpha=1.0)
            # regressory = Ridge(alpha=1.0)
            # regressorx = KernelRidge(alpha=1.0,kernel='rbf')
            # regressory = KernelRidge(alpha=1.0,kernel='rbf')
            # regressorx = ElasticNet(alpha=1.0, l1_ratio=0.5)
            # regressory = ElasticNet(alpha=1.0, l1_ratio=0.5)
            # regressorx = GaussianHMM(n_components=2)
            # regressorx.fit(X_train)
            # hidden_state = regressorx.predict(X_train)
            # pred = regressorx.means_[hidden_state]
            # print(pred.shape)
            
            
            # regressorx.fit(X_train, y_trainx)
            # regressory.fit(X_train, y_trainy)
            # y_predx = regressorx.predict(X_test)
            # y_predy = regressory.predict(X_test)
            
            coefficientsx = np.polyfit(X_train, y_trainx,4)
            coefficientsy = np.polyfit(X_train, y_trainy,4)
            
            polynomialx = np.poly1d(coefficientsx)
            polynomialy = np.poly1d(coefficientsy)
            y_predx = polynomialx(X_test)
            y_predy = polynomialy(X_test)
            pred_point = Marker()
            pred_point.header.frame_id = "map"
            pred_point.header.stamp = rospy.Time.now()  
            pred_point.ns = "traj_pred"
            pred_point.type = Marker.LINE_STRIP
            pred_point.id = numob
            pred_point.scale.x = 0.1
            pred_point.scale.y = 0.1
            pred_point.scale.z = 0.1
            pred_point.color.a = 1.0
            pred_point.color.r = 1.0
            pred_point.color.g = 0.0
            pred_point.color.b = 0.0
            
            for i in range(30):
                p1 = Point()
                p1.x = y_predx[i]
                p1.y = y_predy[i]
                # p1.x = pred[i,0]
                p1.z = 0.0
                pred_point.points.append(p1)
            self.pred_traj.markers.append(pred_point)
                    # pred_traj = MarkerArray()
        

def main():
    rospy.init_node("predict")
    traj_predict()
    rospy.spin()
    
if __name__=="__main__":
    main()     