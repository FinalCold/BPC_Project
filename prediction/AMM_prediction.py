# -*- coding: utf-8 -*-

import numpy as np
from scipy import interpolate
import sys
import scipy.stats
from data_parser import DataParser
from kalman_filter import KalmanFilter
import itertools
# from visualizer import *

class Predictor:
    def __init__(self, global_pose_dict, current_frame_id):
        self.global_pose_dict = global_pose_dict
        self.current_frame_id = current_frame_id
        self.period_of_predict = 0.1
        self.predict_dict = {}
        
        self.AMM_prob_init_dict = {'CV': 0.99, 
                              'CTA_a1': 0.00125, 'CTA_d1': 0.00125,
                              'CTA_a2': 0.00125, 'CTA_d2': 0.00125,
                              'CT_l1': 0.00125, 'CT_r1': 0.00125,
                              'CT_l2': 0.00125, 'CT_r2': 0.00125
                              } 
        self.AMM_prob_dict = {'CV': 0.99, 
                              'CTA_a1': 0.00125, 'CTA_d1': 0.00125,
                              'CTA_a2': 0.00125, 'CTA_d2': 0.00125,
                              'CT_l1': 0.00125, 'CT_r1': 0.00125,
                              'CT_l2': 0.00125, 'CT_r2': 0.00125
                              } 
        
    def predict(self, period_of_predict):
        self.period_of_predict = period_of_predict
        # print(period_of_predict)
        interpolated_track_dict = self.spline_interpolate()
        self.AMM_predict(interpolated_track_dict)
            
    def AMM_predict(self, interpolated_track_dict, x_in = np.mat([[0.0, 0.0, 0.0, 0.0]]).transpose()):
        # ========== parameter setting ========== #
        x_in = np.mat([[0.0, 0.0, 0.0, 0.0]]).transpose()
        # print('x_in',x_in)
        P_in = np.mat(  [[100.0,   0,     0,     0    ],
                         [0,     100.0, 0,     0    ],
                         [0,     0,     100.0,   0    ],
                         [0,     0,     0,     100.0]])
        
        H_in = np.mat([[1,0,0,0], [0,0,1,0]])
        R_in = np.mat([[0.1, 0],[0, 0.1]])




        ## tuning ?    
        t = 0.5    
        t_2 = t*t
        t_3 = t_2 * t
        t_4 = t_3 * t
        
        alpha_ax_2 = 2.25 * 2.25
        alpha_ay_2 = 2.25 * 2.25
        
        Q_in = np.mat(  [[t_4/4 * alpha_ax_2,   t_3/2 * alpha_ax_2,     0,                     0                 ],
                         [t_3/2 * alpha_ax_2,   t_2 * alpha_ax_2,       0,                     0                 ],
                         [0,                    0,                      t_4/4 * alpha_ay_2,    t_3/2 * alpha_ay_2],
                         [0,                    0,                      t_3/2 * alpha_ay_2,    t_2 * alpha_ay_2  ]]) 
                         
        # ========== parameter setting ========== #

        # print('interpolated_track_dict', interpolated_track_dict)

        for track_id in interpolated_track_dict.keys():
            track_ids = []

            # print('track_id', track_id)
            
            track_ids.append(track_id)
            history_trajectory_x = interpolated_track_dict[track_id][0]
            # print('history_trajectory_x',history_trajectory_x)
            history_trajectory_y = interpolated_track_dict[track_id][1]
            # print('history_trajectory_y', history_trajectory_y)
            
            #x_in = np.mat([[history_trajectory_x[0], history_trajectory_y[0], 0.0, 0.0]]).transpose()
            
            kf0 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf1 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf2 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf3 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf4 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf5 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf6 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf7 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in) 
            kf8 = KalmanFilter(x_in, P_in, H_in, R_in, Q_in)
            
            for i in range(len(history_trajectory_x) - 1, -1, -1):

                #===================== estimation done =================== #
                pose_xy = np.mat([history_trajectory_x[i],history_trajectory_y[i]])
                
                # print('pose_xy', pose_xy, i)
                
                kf0.predict('CV',t)
                kf0.update(pose_xy.transpose()) #need to transpose?
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf0.S_)

                L_cv = gaussain_distribution.pdf(kf0.y_.transpose())
                L_cv = max(1e-30,L_cv)

                kf1.predict('CTA_a1',t)
                kf1.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf1.S_)
                L_cta_a1 = gaussain_distribution.pdf(kf1.y_.transpose())
                L_cta_a1 = max(1e-30,L_cta_a1)
                
                kf2.predict('CTA_a2',t)
                kf2.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf2.S_)
                L_cta_a2 = gaussain_distribution.pdf(kf2.y_.transpose())
                L_cta_a2 = max(1e-30,L_cta_a2)
                
                kf3.predict('CTA_d1',t)
                kf3.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf3.S_)
                L_cta_d1 = gaussain_distribution.pdf(kf3.y_.transpose())
                L_cta_d1 = max(1e-30,L_cta_d1)                
                
                kf4.predict('CTA_d2',t)
                kf4.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf4.S_)
                L_cta_d2 = gaussain_distribution.pdf(kf4.y_.transpose())   
                L_cta_d2 = max(1e-30,L_cta_d2)   

                kf5.predict('CT_l1',t)
                kf5.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf5.S_)
                L_ct_l1 = gaussain_distribution.pdf(kf5.y_.transpose())  
                L_ct_l1 = max(1e-30,L_ct_l1) 
                
                kf6.predict('CT_l2',t)
                kf6.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf6.S_)
                L_ct_l2 = gaussain_distribution.pdf(kf6.y_.transpose())
                L_ct_l2 = max(1e-30,L_ct_l2) 
                
                kf7.predict('CT_r1',t)
                kf7.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf7.S_)
                L_ct_r1 = gaussain_distribution.pdf(kf7.y_.transpose())
                L_ct_r1 = max(1e-30,L_ct_r1) 
                
                kf8.predict('CT_r2',t)
                kf8.update(pose_xy.transpose())
                gaussain_distribution = scipy.stats.multivariate_normal([0,0], kf8.S_)
                L_ct_r2 = gaussain_distribution.pdf(kf8.y_.transpose())
                L_ct_r2 = max(1e-30,L_ct_r2) 
                
                prob_sum = self.AMM_prob_dict['CV'] * L_cv + \
                        self.AMM_prob_dict['CTA_a1'] * L_cta_a1 + self.AMM_prob_dict['CTA_a2'] * L_cta_a2 + \
                        self.AMM_prob_dict['CTA_d1'] * L_cta_d1 + self.AMM_prob_dict['CTA_d2'] * L_cta_d2 + \
                        self.AMM_prob_dict['CT_l1'] * L_ct_l1 + self.AMM_prob_dict['CT_l2'] * L_ct_l2 + \
                        self.AMM_prob_dict['CT_r1'] * L_ct_r1 + self.AMM_prob_dict['CT_r2'] * L_ct_r2
  
                #print("+++++++++++++++++++++")        
                #print(self.AMM_prob_dict)
                
                self.AMM_prob_dict['CV'] = self.AMM_prob_dict['CV'] * L_cv / prob_sum
                self.AMM_prob_dict['CTA_a1'] = self.AMM_prob_dict['CTA_a1'] * L_cta_a1 / prob_sum
                self.AMM_prob_dict['CTA_a2'] = self.AMM_prob_dict['CTA_a2'] * L_cta_a2 / prob_sum
                self.AMM_prob_dict['CTA_d1'] = self.AMM_prob_dict['CTA_d1'] * L_cta_d1 / prob_sum
                self.AMM_prob_dict['CTA_d2'] = self.AMM_prob_dict['CTA_d2'] * L_cta_d2 / prob_sum
                
                self.AMM_prob_dict['CT_l1'] = self.AMM_prob_dict['CT_l1'] * L_ct_l1 / prob_sum                             
                self.AMM_prob_dict['CT_l2'] = self.AMM_prob_dict['CT_l2'] * L_ct_l2 / prob_sum
                self.AMM_prob_dict['CT_r1'] = self.AMM_prob_dict['CT_r1'] * L_ct_r1 / prob_sum                 
                self.AMM_prob_dict['CT_r2'] = self.AMM_prob_dict['CT_r2'] * L_ct_r2 / prob_sum                
                                                                                                            
                
                current_x_estimation = self.AMM_prob_dict['CV'] * kf0.x_ + \
                        self.AMM_prob_dict['CTA_a1'] * kf1.x_ + self.AMM_prob_dict['CTA_a2'] * kf2.x_ + \
                        self.AMM_prob_dict['CTA_d1'] * kf3.x_ + self.AMM_prob_dict['CTA_d2'] * kf4.x_ + \
                        self.AMM_prob_dict['CT_l1'] * kf5.x_ + self.AMM_prob_dict['CT_l2'] * kf6.x_ + \
                        self.AMM_prob_dict['CT_r1'] * kf7.x_ + self.AMM_prob_dict['CT_r2'] * kf8.x_

                # print('current_x_estimation', current_x_estimation)
                

                current_P_estimation = self.AMM_prob_dict['CV'] * (kf0.P_ +  (current_x_estimation - kf0.x_) * (current_x_estimation - kf0.x_).transpose()) + \
                        self.AMM_prob_dict['CTA_a1'] * (kf1.P_ + (current_x_estimation - kf1.x_) * (current_x_estimation - kf1.x_).transpose()) + \
                        self.AMM_prob_dict['CTA_a2'] * (kf2.P_ + (current_x_estimation - kf2.x_) * (current_x_estimation - kf2.x_).transpose()) + \
                        self.AMM_prob_dict['CTA_d1'] * (kf3.P_ + (current_x_estimation - kf3.x_) * (current_x_estimation - kf3.x_).transpose()) +\
                        self.AMM_prob_dict['CTA_d2'] * (kf4.P_ + (current_x_estimation - kf4.x_) * (current_x_estimation - kf4.x_).transpose()) + \
                        self.AMM_prob_dict['CT_l1'] * (kf5.P_ + (current_x_estimation - kf5.x_) * (current_x_estimation - kf5.x_).transpose()) + \
                        self.AMM_prob_dict['CT_l2'] * (kf6.P_ + (current_x_estimation - kf6.x_) * (current_x_estimation - kf6.x_).transpose()) + \
                        self.AMM_prob_dict['CT_r1'] * (kf7.P_ + (current_x_estimation - kf7.x_) * (current_x_estimation - kf7.x_).transpose()) + \
                        self.AMM_prob_dict['CT_r2'] * (kf8.P_ + (current_x_estimation - kf8.x_) * (current_x_estimation - kf8.x_).transpose())
                #===================== estimation done =================== #
  
            # now lets predict:
            
            current_pose_xy = np.mat([history_trajectory_x[0],history_trajectory_y[0]])
            
            # print('current_pose_xy',current_pose_xy)

            kf0.predict('CV',self.period_of_predict)
            kf1.predict('CTA_a1',self.period_of_predict)
            kf2.predict('CTA_a2',self.period_of_predict)
            kf3.predict('CTA_d1',self.period_of_predict)
            kf4.predict('CTA_d2',self.period_of_predict)               
            kf5.predict('CT_l1',self.period_of_predict)
            kf6.predict('CT_l2',self.period_of_predict)
            kf7.predict('CT_r1',self.period_of_predict)
            kf8.predict('CT_r2',self.period_of_predict)

            prob_sum_p = self.AMM_prob_dict['CV'] + \
                         self.AMM_prob_dict['CTA_a1'] + self.AMM_prob_dict['CTA_a2'] + \
                         self.AMM_prob_dict['CTA_d1'] + self.AMM_prob_dict['CTA_d2'] + \
                         self.AMM_prob_dict['CT_l1']  + self.AMM_prob_dict['CT_l2']  + \
                         self.AMM_prob_dict['CT_r1']  + self.AMM_prob_dict['CT_r2']  + 0.00000000000001
                         
            self.AMM_prob_dict['CV'] = self.AMM_prob_dict['CV'] / prob_sum_p
            self.AMM_prob_dict['CTA_a1'] = self.AMM_prob_dict['CTA_a1']  / prob_sum_p
            self.AMM_prob_dict['CTA_a2'] = self.AMM_prob_dict['CTA_a2']  / prob_sum_p
            self.AMM_prob_dict['CTA_d1'] = self.AMM_prob_dict['CTA_d1']  / prob_sum_p
            self.AMM_prob_dict['CTA_d2'] = self.AMM_prob_dict['CTA_d2']  / prob_sum_p
                
            self.AMM_prob_dict['CT_l1'] = self.AMM_prob_dict['CT_l1']  / prob_sum_p                            
            self.AMM_prob_dict['CT_l2'] = self.AMM_prob_dict['CT_l2']  / prob_sum_p
            self.AMM_prob_dict['CT_r1'] = self.AMM_prob_dict['CT_r1']  / prob_sum_p                
            self.AMM_prob_dict['CT_r2'] = self.AMM_prob_dict['CT_r2']  / prob_sum_p
                
            predict_x_estimation = self.AMM_prob_dict['CV'] * kf0.x_ + \
                        self.AMM_prob_dict['CTA_a1'] * kf1.x_ + self.AMM_prob_dict['CTA_a2'] * kf2.x_ + \
                        self.AMM_prob_dict['CTA_d1'] * kf3.x_ + self.AMM_prob_dict['CTA_d2'] * kf4.x_ + \
                        self.AMM_prob_dict['CT_l1'] * kf5.x_ + self.AMM_prob_dict['CT_l2'] * kf6.x_ + \
                        self.AMM_prob_dict['CT_r1'] * kf7.x_ + self.AMM_prob_dict['CT_r2'] * kf8.x_

            predict_P_estimation = self.AMM_prob_dict['CV'] * (kf0.P_ +  (current_pose_xy - kf0.x_) * (current_pose_xy - kf0.x_).transpose()) + \
                        self.AMM_prob_dict['CTA_a1'] * (kf1.P_ + (current_x_estimation - kf1.x_) * (current_x_estimation - kf1.x_).transpose()) + \
                        self.AMM_prob_dict['CTA_a2'] * (kf2.P_ + (current_x_estimation - kf2.x_) * (current_x_estimation - kf2.x_).transpose()) + \
                        self.AMM_prob_dict['CTA_d1'] * (kf3.P_ + (current_x_estimation - kf3.x_) * (current_x_estimation - kf3.x_).transpose()) +\
                        self.AMM_prob_dict['CTA_d2'] * (kf4.P_ + (current_x_estimation - kf4.x_) * (current_x_estimation - kf4.x_).transpose()) + \
                        self.AMM_prob_dict['CT_l1'] * (kf5.P_ + (current_x_estimation - kf5.x_) * (current_x_estimation - kf5.x_).transpose()) + \
                        self.AMM_prob_dict['CT_l2'] * (kf6.P_ + (current_x_estimation - kf6.x_) * (current_x_estimation - kf6.x_).transpose()) + \
                        self.AMM_prob_dict['CT_r1'] * (kf7.P_ + (current_x_estimation - kf7.x_) * (current_x_estimation - kf7.x_).transpose()) + \
                        self.AMM_prob_dict['CT_r2'] * (kf8.P_ + (current_x_estimation - kf8.x_) * (current_x_estimation - kf8.x_).transpose())
            
            self.predict_dict[track_id] = [predict_x_estimation, predict_P_estimation]

            # print('predict_dict[',track_id,']',predict_x_estimation)



    def spline_interpolate(self):

        min_appears_hits = 3          
        
        reference_frame_num = 10          
  
        tmp = min(reference_frame_num, self.current_frame_id)

        # print([reference_frame_num, self.current_frame_id])

        frames = []
        try:
            for i in range(tmp):
                # print('self.current_frame_id - i ', self.current_frame_id - i )
                frames.append(self.global_pose_dict[self.current_frame_id - i ])
        except:
            pass
            # print('no frame')
        # frames = [self.global_pose_dict[self.current_frame_id - i] for i in range(tmp) ]

        frame_current = frames[0]
        # print('frame_current', frame_current)
        
        intersection_track_ids = set()
        # =========================== begin ============================= #
        # only when a track appears > min_appears_hits in last 5 frames, 
        # and it's appears on current frame , we apply interpolation
        track_id_counter = {}
        for frame in frames:
            for track in frame.keys():
                if track not in track_id_counter.keys():
                    track_id_counter[track] = 1
                else:
                    track_id_counter[track] += 1
        
        interpolate_track_list = []            
        for track_id in track_id_counter.keys():
            if (track_id_counter[track_id] >= min_appears_hits) and (track_id in frame_current.keys()):
                
                interpolate_track_list.append(track_id)
        # ============================= end =========================== #
        
        interpolate_dict = {}
        
        for track_id in interpolate_track_list:
            pX_list = []
            pY_list = []
            t = []
            for index,frame in enumerate(frames):
                if (track_id in frame.keys()):
                    pX_list.append(frame[track_id][2][0])
                    # print('id, px',track_id, frame[track_id][2][0])
                    pY_list.append(frame[track_id][2][1])
                    # print('id, py',track_id, frame[track_id][2])
                    t.append(index)
                    
            pX = np.array(pX_list)
            pY = np.array(pY_list)
            
            N = t[-1] + 1
            pXY_after_interpolate = self.interpcurve(N,pX,pY)
            interpolate_dict[track_id] = [pXY_after_interpolate[:,0],pXY_after_interpolate[:,1]]
        return interpolate_dict
        

    def interpcurve(self,N,pX,pY):
        #equally spaced in arclength
        N=np.transpose(np.linspace(0,1,N))

        nt=N.size
        n=pX.size
        pxy=np.array((pX,pY)).T
        p1=pxy[0,:]
        pend=pxy[-1,:]
        last_segment= np.linalg.norm(np.subtract(p1,pend))
        epsilon= 10*np.finfo(float).eps
        
        pt=np.zeros((nt,2))

        chordlen = (np.sum(np.diff(pxy,axis=0)**2,axis=1))**(1/2)
        chordlen = chordlen/np.sum(chordlen)
        cumarc = np.append(0,np.cumsum(chordlen))

        tbins= np.digitize(N,cumarc) 
        tbins[np.where(np.bitwise_or((tbins<=0),(N<=0)))] = 1
        tbins[np.where(np.bitwise_or((tbins>=n),(N>=1)))] = n - 1    
        
        s = np.divide((N - cumarc[tbins]),chordlen[tbins-1])
        pt = pxy[tbins,:] + np.multiply((pxy[tbins,:] - pxy[tbins-1,:]),(np.vstack([s]*2)).T)

        return pt 


if __name__ == "__main__":

    print('--------------------processing--------------------')

    last_range = 5
    pre_period = 1

    perdict_range = list(np.arange(1, last_range+1, pre_period))

    # print('predict range : ', 1, ',', last_range)
    # print('predict period : ', pre_period)

    p1 = DataParser("./data/0000.txt", data_type = 'lidar')
    p1.parse()

    file_name = p1.file_name[-8:-4] 
    print_str = 'input file : %s.txt  \r' % (file_name)
    # print(print_str)

    result = []

    # print(p1.result_dict)

    total_count = len(list(p1.result_dict.keys())) * len(perdict_range)

    count = 0
    

    # print(p1.result_dict[15][15][2])

    # print(p1.result_dict.keys())
    last_frame = list(p1.result_dict.keys())[-1]
    print('last frame : ', last_frame)

    

    # for frame in range(15,16):
    for frame in p1.result_dict.keys():
        # print('frame',frame)

        if frame == 0:
            pass

        else:

            predictor = Predictor(p1.result_dict, current_frame_id = frame)
            predictor.predict(period_of_predict = 0)
            dict_keys = list(predictor.predict_dict.keys())
            # print(dict_keys)

            count_1 = 0
            # print("start1: %s"%count_1)    

            for period in perdict_range:
                # print('period',period)
                
                predictor.predict(period_of_predict = period)
                # print(predictor.predict(period_of_predict = period))
                count += 1
                processed = count*100/total_count
                print_str = 'frame : %d  period %d  processed %f%s  \r' % (frame, period, processed, '%')
                sys.stdout.write(print_str)
                sys.stdout.flush()

                
                # for key in [5]:
                for key in dict_keys:
                    # print('key',key)
                    # print("start2: %s"%count_1)    
                    # print('current_pose/id : ', key, p1.result_dict[frame][key][2])
                    # print('predictor.predict_dict','[',key,']', predictor.predict_dict[key])
                    present = p1.result_dict[frame][key][2]
                    present = [frame, key] + present
                    present.insert(4, -1.5)
                    del present[5]

                    predicted = predictor.predict_dict[key][0].tolist()
                    predicted = list(itertools.chain(*predicted))
                    predicted = [frame, key] + predicted
                    py = predicted.pop(4)
                    predicted.insert(3, py)
                    del predicted[4:6]
                    predicted.insert(4, -1.5)
                    
                    # print('예측',predicted)


                    if count_1 == 0 :
                        result.append(present) 
                        # print('현재',present)
         
                    else :
                        pass
                    
                    # print(present)
                    
                    '''
                    if predicted[1]  == 15 :

                        result.append(predicted)
                    else : 
                        pass
                    '''
                    result.append(predicted)

                count_1 += 1
                # print("count: %s"%count_1)

                # predictor.predict(period_of_predict = 0)
                # dict_keys = list(predictor.predict_dict.keys())
        # print('result', result)

    pre_period = str(pre_period).replace('.', '')
    

    with open('predict_result_AB3DMOT_'+ str(last_range) + '_' + str(pre_period) + '_' + file_name + '.txt', 'w') as f:

        for data in result:

            data_conv = str(data).replace('[','').replace(']','').replace(',','') + '\n'
            # print(data_conv)
            
            ## file save
            f.writelines(data_conv)

        print('file saved as', 'predict_result_AB3DMOT_'+ str(last_range) + '_' + str(pre_period) + '_' + file_name  + '.txt'  )
        print("data format :  'frame’, ‘id’, ‘tx’, ‘ty’,")
        print('----------------------finised----------------------')
    # arr = np.array(result)
    # print(arr)


