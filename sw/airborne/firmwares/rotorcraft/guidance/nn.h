#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

typedef struct  {

   double*** weights;
   double*** biases;
   double** norms;
   double** minmax;
   int input_len;
   int output_len;
   int num_layers;
   int hidden_units;
} network ;  


// typedef enum { false, true } bool;
#define MAX(x, y) (((x) > (y)) ? (x) : (y))

/*!
 * Just the Rectified Linear activation function
 */
double relu(double a);

/*!
 * Computes the output of a layer
 *
 *  @param input        array [input_len] of activations of the previous layer 
 *  @param output       array [input_len] to store the activations of the current layer 
 *  @param n            network
 *  @param l            layer to compute
 *  @param input_len           
 *  @param output_len            
 *  @param nl           activation function 
 * 
 */
void nn_layer(double* input, double* output, network n, int l, int input_len, int output_len, double(*nl)(double));

/*!
 * Prepares the input of the network
 *
 *  @param input        array [input_len]  
 *  @param n            network   
 */
int preprocess(double* input, network n);

/*!
 * Computes the control in the real units from the output of the network 
 *
 *  @param output       array [output_len] 
 *  @param n            network
 */
int postprocess(double* output, network n);
/*!
 * Computes the output of a network, all hidden layers must contain the same number of units
 *
 *      ---TODO pre/post processing ---
 *
 *
 *  @param input        array [input_len] of activations of the previous layer 
 *  @param output       array [input_len} to store the activations of the current layer 
 *  @param n            network
 *  @param nl           activation function 
 * 
 */
void nn(double* input, double* output, network n);

/*!
 * Reads the weight and bias matrices from a file as well as the 
 *  number of inputs, outputs, layers and units per hidden layer
 * Memory for the weights and biases is allocated, remember to free it
 *
 *    
 *  - First line: L#layers
 *  - Each weights/biases matrix starts with: W/b#layer_id, #outputs, #inputs
 *         A row per output, a column per input
 *   
 *
 *      ---TODO pre/post processing ---
 *
 *  @param filename     filename of network description
 *  @param n          pointer to network  
 */
int nn_read(char* filename,network* n);

/*!
 * Free the memory allocated for the network
 *
 * @param n
 */
int free_network(network* n);
