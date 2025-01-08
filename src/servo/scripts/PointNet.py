import pointnet2_sem_seg
import torch




if __name__ == '__main__':
    classes = ['cylinder','box','floor', 'back', 'ceiling', 'side']
    NUM_CLASSES = 6

    '''MODEL LOADING'''       
    classifier = pointnet2_sem_seg.get_model(NUM_CLASSES).cuda()
    # import pdb; pdb.set_trace()
    checkpoint = torch.load('/home/jetson/catkin_ws/src/servo/scripts/best_model.pth') #torch.load(str(experiment_dir) + '/checkpoints/best_model.pth')
    classifier.load_state_dict(checkpoint['model_state_dict'])
    classifier = classifier.eval()    