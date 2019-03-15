from keras.models import Model
from keras.layers import Dense, Flatten, Input, merge, Lambda
from keras.layers.merge import concatenate
from keras.initializers import normal, RandomNormal
import tensorflow as tf
import keras.backend as K


class ActorNetwork(object):
    def __init__(self, sess, state_size, action_size, BATCH_SIZE, TAU, LEARNING_RATE):
        self.sess = sess
        self.BATCH_SIZE = BATCH_SIZE
        self.TAU = TAU
        self.LEARNING_RATE = LEARNING_RATE

        K.set_session(sess)

        #Now create the model
        self.model , self.weights, self.state = self.create_actor_network(state_size, action_size)
        self.target_model, self.target_weights, self.target_state = self.create_actor_network(state_size, action_size)
        self.action_gradient = tf.placeholder(tf.float32,[None, action_size])
        self.params_grad = tf.gradients(self.model.output, self.weights, -self.action_gradient)
        grads = zip(self.params_grad, self.weights)
        self.optimize = tf.train.AdamOptimizer(LEARNING_RATE).apply_gradients(grads)
        self.sess.run(tf.global_variables_initializer())

    def train(self, states, action_grads):
        self.sess.run(self.optimize, feed_dict={
            self.state: states,
            self.action_gradient: action_grads
        })

    def target_train(self):
        actor_weights = self.model.get_weights()
        actor_target_weights = self.target_model.get_weights()
        for i in xrange(len(actor_weights)):
            actor_target_weights[i] = self.TAU * actor_weights[i] + (1 - self.TAU)* actor_target_weights[i]
        self.target_model.set_weights(actor_target_weights)

    def create_actor_network(self, state_size, action_dim):
        print("Now we build the model ActorV2")
        S = Input(shape=[state_size], name='InputLayer', dtype='float32')
        h0 = Dense(1024, activation='relu', kernel_initializer=RandomNormal(stddev=1e-4), name='HiddenLayer0')(S)
        h1 = Dense(512, activation='relu', kernel_initializer=RandomNormal(stddev=1e-4), name='HiddenLayer1')(h0)
        h2 = Dense(512, activation='relu', kernel_initializer=RandomNormal(stddev=1e-4), name='HiddenLayer2')(h1)

        # to create a final layer with maybe different activation functions
        # create a layer for each activation function
        # tanh is in range of -1 to 1

        omega_left_wheel = Dense(1, activation='tanh', kernel_initializer=RandomNormal(stddev=1e-4), name='action_left_wheel')(h2)
        omega_right_wheel = Dense(1, activation='tanh', kernel_initializer=RandomNormal(stddev=1e-4), name='action_right_wheel')(h2)

        # and merge them later
        #V = merge([omega_left_wheel, omega_right_wheel], mode='concat', name='OutputLayer')
        V = concatenate([omega_left_wheel, omega_right_wheel], name='OutputLayer')

        # define the model from layers
        model = Model(inputs=S, outputs=V, name='ActorModel')

        return model, model.trainable_weights, S

