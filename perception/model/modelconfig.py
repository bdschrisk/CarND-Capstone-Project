### Model Configuration File ###

### Model config ###
# models path
model_path = "./model/checkpoints/"
# model checkpoint path pattern, used for restoring models and saving checkpoints in training
checkpoint_pattern = "checkpoint-*.h5"
model_config_name = "DenseNet-model.json"
model_weights_name = "DenseNet-weights.h5"

output_size = 2
img_dim = (None, None, 3)
input_dim = (48, 48, 3)

### Network params ###
dropout = 0.2
depth = 20
growth_rate = 12
dense_blocks = 3
filters = 16
weight_decay = 1e-4

labels = {
        'unknown' : 0,
        'off' : 0,
#
        'green' : 2,
        #'greenstraight' : 2 | 8,
        #'greenleft' : 2 | 16,
        #'greenright' : 2 | 32,
        #'greenstraightleft' : 2 | 8 | 16,
        #'greenstraightright' : 2 | 8 | 32,
#
        'yellow' : 1,
        #'yellowleft' : 1 | 16,
#
        'red' : 4 | 8,
        #'redleft' : 4 | 16,
        #'redright' : 4 | 32,
        #'redstraightleft' : 4 | 8 | 16,
        #'redstraightright' : 4 | 8 | 32
    }