#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "nn.h"

double relu(double a)
{
    return MAX(0, a);
}

void nn_layer(double* input, double* output, network n, int l, int input_len, int output_len, double(*nl)(double))
{
    memset (output, 0, output_len*sizeof(double));

    for(int i=0; i < output_len; i+=1)
    {
        for(int j=0; j < input_len; j+=1)
        {
            output[i] += n.weights[l][i][j] * input[j];
        }

        output[i] += n.biases[l][i][0];
        output[i] = (*nl)(output[i]);
    }
};

int preprocess(double* input, network n)
{
    //Apply normalization
    for(int i=0; i<n.input_len; i++)
    {
        input[i] = (input[i]-n.norms[i][0])/ n.norms[i][1];
    }
}

int postprocess(double* output, network n)
{
    for(int i=0; i<n.output_len; i++)
    {
        output[i] =  (((output[i]/2.0)+0.5) * (n.minmax[i][1] - n.minmax[i][0]) + n.minmax[i][0]);
    }

    //Reverse normalization
    for(int i=0; i<(n.output_len); i++)
    {
        output[i] = (output[i] * n.norms[n.input_len+i][1]) + n.norms[n.input_len+i][0]    ;
    }

}

void nn(double* input, double* output, network n)
{
    // make a copy of input:
    double* input_copy = malloc(n.input_len * sizeof(double) );
    memcpy(input_copy, input, n.input_len * sizeof(double));

    double* prev_layer_data = input_copy;
    double* next_layer_data = malloc(sizeof(double) * n.hidden_units);

    int prev_layer_units = n.input_len;
    int next_layer_units = n.hidden_units;

    double(*nl)(double);
    (nl) = relu;

    preprocess(prev_layer_data, n);

    for(int i=0; i<n.num_layers; i+=1)
    {
        if (i == n.num_layers-1)
        {
            free(next_layer_data);
            next_layer_units = n.output_len;
            next_layer_data = output;
            nl = tanh;
        }
        nn_layer(prev_layer_data, next_layer_data, n, i, prev_layer_units, next_layer_units, nl);
        if (i==0)
        {
          free(input_copy);
          prev_layer_data = malloc(sizeof(double) * n.hidden_units);
        }
        // To avoid the continous allocation of memory, prev/next_layer date are
        // swapped for every layer
        double* tmp = prev_layer_data;
        prev_layer_data = next_layer_data;
        next_layer_data = tmp;
        prev_layer_units = next_layer_units;

    }

    free(next_layer_data);

    postprocess(output, n);
}

int nn_read(char* nn_filename,network* n)
{
    FILE* nn_w_file = fopen(nn_filename, "r");

    char nn_a;
    int nn_num_layers;

//     Read number of layers to load layers*2 matrices
    fscanf(nn_w_file, "%c", &nn_a);
    if(nn_a!='L')
    {
        printf("Wrong format");
        return -1;
    }
    fscanf(nn_w_file, "%d%c", &nn_num_layers, &nn_a);
    n->num_layers = nn_num_layers;

    n->weights = malloc(sizeof(double **) * nn_num_layers);
    n->biases = malloc(sizeof(double **) * nn_num_layers);

    for(int l=0; l < 2 * nn_num_layers; l++)
    {
        int layer;
        int rows;
        int columns;
        double*** store_to;
        fscanf(nn_w_file, "%c", &nn_a, &layer);

        //If line starts with W: new weights matrix
        //If                  b: new biases matrix
        switch(nn_a)
        {
            case 'W': store_to =  n->weights;
                      break;
            case 'b': store_to = n->biases;
                      break;
            default:
                printf("Wrong format\n");
        }
        fscanf(nn_w_file, "%d,", &layer);
        fscanf(nn_w_file, "%d,", &rows);
        fscanf(nn_w_file, "%d\n", &columns);

        //Number of inputs, outputs, hidden units
        if(l ==0)
        {
            n->input_len = columns;
            n->hidden_units = rows;
        }
        if(l==nn_num_layers-1)
            n->output_len = columns;

        //Allocate memory and read this matrix
        store_to[(int)(l/2)] = malloc(rows * sizeof(double *));
        double w;
        for(int r=0; r < rows; r++)
        {
            store_to[l/2][r] = malloc( columns * sizeof(double));
            for(int c=0; c < columns; c++)
            {
                fscanf(nn_w_file, "%lf\t", &w);
                store_to[l/2][r][c] = w;
            }
            fscanf(nn_w_file, "\n", NULL);
        }
    }

    n->norms = malloc(sizeof(double *) * (n->input_len+n->output_len));
    n->minmax = malloc(sizeof(double *) *  n->output_len );

    fscanf(nn_w_file, "%c\n", &nn_a);
    if(nn_a!='I')
    {
        printf("Wrong format");
        return -1;
    }
    for(int i=0; i < n->input_len; i++)
    {
        double mean=0;
        double std=0;
        n->norms[i] = malloc(2 * sizeof (double));
        fscanf(nn_w_file, "%lf\t", &mean);
        n->norms[i][0] = mean;
        fscanf(nn_w_file, "%lf\n", &std);
        n->norms[i][1] = std;
    }

    fscanf(nn_w_file, "%c\n", &nn_a);
    if(nn_a!='O')
    {
        printf("Wrong format");
        return -1;
    }
    for(int i=n->input_len; i < (n->input_len+n->output_len); i++)
    {
        double mean=0;
        double std=0;
        n->norms[i] = malloc(2 * sizeof (double));
        fscanf(nn_w_file, "%lf\t", &mean);
        n->norms[i][0] = mean;
        fscanf(nn_w_file, "%lf\n", &std);
        n->norms[i][1] = std;
    }

    fscanf(nn_w_file, "%c\n", &nn_a);
    if(nn_a!='B')
    {
        printf("Wrong format");
        return -1;
    }
    for(int i=0; i < (n->output_len); i++)
    {
        double min=0;
        double max=0;
        n->minmax[i] = malloc(2 * sizeof (double));
        fscanf(nn_w_file, "%lf\t", &min);
        n->minmax[i][0] = min;
        fscanf(nn_w_file, "%lf\n", &max);
        n->minmax[i][1] = max;
    }

    fclose(nn_w_file);
    printf("Network loaded: %d layers with %d units, %d inputs, %d outputs\n", n->num_layers, n->hidden_units, n->input_len, n->output_len) ;

    return 0;
}


int free_network(network* n)
{


    for(int i=0; i < (n->input_len+n->output_len); i++)
    {
        free(n->norms[i]);
    }
    for(int i=0; i < (n->output_len); i++)
    {
        free(n->minmax[i]);
    }

    free(n->norms);
    free(n->minmax);


    for(int i=0; i<n->num_layers-1; i+=1)
    {
        for(int j=0; j<n->hidden_units; j+=1)
        {
            free(n->weights[i][j]);
            free(n->biases[i][j]);
        }
        free(n->weights[i]);
        free(n->biases[i]);
    }
    for(int j=0; j<n->output_len; j+=1)
    {
        free(n->weights[n->num_layers-1][j]);
        free(n->biases[n->num_layers-1][j]);
    }
    free(n->weights[n->num_layers-1]);
    free(n->biases[n->num_layers-1]);
    free(n->weights);
    free(n->biases);

    return 0;
}



