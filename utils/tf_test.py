import tensorflow as tf
import time

config = tf.ConfigProto()
config.allow_soft_placement = True  # allow placing ops on cpu if they are not fit for gpu
config.gpu_options.allow_growth = True  # allow the gpu memory allocated for the worker to grow if needed
config.gpu_options.per_process_gpu_memory_fraction = 0.01
config.intra_op_parallelism_threads = 1
config.inter_op_parallelism_threads = 1

with tf.Session(config=config) as sess:
    with tf.device("/gpu:0"):
        a = tf.constant([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], shape=[2, 3], name='a')
        b = tf.constant([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], shape=[3, 2], name='b')
        c = tf.matmul(a, b)
        print ("Ready to calculate")
    print (sess.run(c))

# time.sleep(10)
