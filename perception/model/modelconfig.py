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
        'off' : 1,

        'green' : 2,
        'yellow' : 4,
        'red' : 8
    }