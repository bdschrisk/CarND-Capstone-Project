import numpy as np

# One hot encode
def one_hot_encode(labels, classes):
    class_labels = list(set(classes))
    one_hot = np.zeros([len(labels), len(class_labels)])
    
    for i in range(len(labels)):
        for j in range(len(class_labels)):
            if (labels[i] == class_labels[j]):
                one_hot[i, j] = 1
                break
    
    return one_hot