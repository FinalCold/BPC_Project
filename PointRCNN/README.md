#TO INFERENCE

#TO RUN
> python eval_rcnn.py --cfg_file cfgs/default.yaml --ckpt PointRCNN.pth --batch_size 1 --eval_mode rcnn --test --set TEST.SPLIT test RPN.LOC_XZ_FINE False
