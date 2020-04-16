# Sensor Fusion Self-Driving Car Course
## Lesson 4: Radar Target Generation and Detection

This repository represents my submission for the Radar section of Udacity's Sensor Fusion Engineer course. It consists of the MATLAB script ´Radar_Target_Generation_and_Detection.m´, a corresponding MATLAB live script (the mlx-file), and this README file.

The paragraphs below highlight the central [CA-CFAR](#glossary) algorithm and supplemental steps. Note that, alongside with CA-CFAR, there are several other CFAR algorithms, such as [OS-CFAR](#glossary), [MAMIS-CFAR](#glossary), and multiple other variations of CA-CFAR.

The central idea of the CFAR-CA algorithm is to average the background noise within a neighborhood of the current 'cell under test' (abreviated by 'CUT'). This neighborhood consists of two rings, an inner ring consisting of the immediate neighborhood of the so-called guarding cells, and an outer ring with the training cells. It is the testing cells in which the noise is sampled and averaged to determine the threshold. This threshold is used to evaluate whether the CUT's value is considered a valid signal or only part of the background noise.

## Implementation Steps for the 2D CFAR Process
The starting point of the CFAR implementation is the so-called Range Doppler Map (RDM) which we have produced in a previous step. The RDM is a matrix of dimensions 512 x 128 and is essentially the two dimensional (fast) fourier transformation of the beat signal.
Note that the dimensions of the RDM can be determined by the expression `size(RDM)`. In our case of course,

    size(RDM) = [512 128].

The algorithm iterates over all of the RDM's cells in the range and Doppler dimensions as follows:

    for i = Tr+Gr+1:Nr/2-(Gr+Tr)
        for j = Td+Gd+1:Nd-(Gd+Td)
            % Averaging is done here...
        end
    end

Here `Gr`, `Gd`, `Tr`, and `Td` are the parameters which describe the neighborhood of the CUT. The parameters `Gr` and `Gd` are the dimensions for the guarding cells in the range dimension and the Doppler dimension, respectively, and `Tr` and `Td` for the training cells in the corresponding dimensions.

Notice that the iteration does not start directly at the edges of the RDM-domain. This is because we need to average over all of the CUT's training cells in order to obtain a reasonable threshold. The [following section](#selection-of-training-guard-cells-and-offset) handles this border case.

The averaging algorithm around a CUT at `(i,j)` is given by the following code:

    noise_level = zeros(1, 1);
    for p = i-(Tr+Gr):i+Tr+Gr
        for q = j-(Td+Gd):j+Td+Gd            
            if (abs(p-i)>Gr || abs(q-j)>Gd)
                noise_level = noise_level + db2pow(RDM(p,q));
                threshold = pow2db( noise_level / ((2*(Tr+Gr)+1)*(2*(Td+Gd)+1)-((2*Gr+1)*(2*Gd+1))) );
            end
        end
    end
    threshold = threshold + offset;
    % Evaluation of the RDM-values is done here...

Notice that the if-clause leads to the omission of the inner guarding cells from the averaging process. In order to be able to *add* the signals, we need to convert the values of the RDM which are stored as *signal strength* (measured in dB) to power values and vice versa by using  `db2pow` and `pow2db`, respectively.

Having calculated `threshold`, we take the value of the cell under test (CUT), i.e. the value of the Range Doppler Map at `(i,j)` and determine if it is above or below this threshold.

    CUT=RDM(i,j);
    if( CUT < threshold )
        output(i,j) = 0;
    else
        output(i,j) = max_T;
    end     

Notice that, as a preparatory measure, the output matrix in which we store our algorithm's results is initialized by calling

    output = zeros(size(RDM)).

To summarize, the CA-CFAR algorithm sweeps over the domain of the Range Doppler Map and, for each cell under test, averages the background noise in a neighborhood, adds a fixed offset value, and determines whether the CUT's signal value is below this threshold or not.


### Selection of Training, Guard Cells, and Offset

We need to determine four parameters for the dimensions of the neighborhood. 
The CUT is surrounded by a concentric patch consisting of the inner guarding cells and outer training cells. The inner guarding cells describe a rectangle of width `2*Gr+1` and height `2*Gd+1`, the outer training cells a rectangle of width `2*(Gr+Tr)+1` and `2*(Gd+Td)+1`.

Our choice for the paramters are `Tr = 10`, `Td = 8`, and `Gr = Gd = 4`. In our case, using the above values for `Tr`, `Gr`, `Td`, and `Gd`, we have

    (2*Gr+1)*(2*Gd+1)-1 = 81 - 1 = 80

guarding cells and therefore

    (2*(Tr+Gr)+1)*(2*(Td+Gd)+1)-((2*Gr+1)*(2*Gd+1))) = 725 - 80 = 645

training cells. We average the sum of the signal values in all the training cells. Finally, we add an offset to this average value which in our case is set to `6`.


### Steps Taken to Suppress the Non-Thresholded Cells at the Edges
In order to suppress the non-thresholded cells at the edges, two nested for-loops iterate over the whole output array, setting the output matrix to zero whenever the elements are within distance of the border:

    for i = 1:Nr/2
        for j = 1:Nd
            if (abs(i)<Tr+Gr || abs(Nr/2-i)<Tr+Gr || abs(j)<Td+Gd || abs(Nd-j)<Td+Gd )
                output(i,j) = 0.0;
            end
        end
    end

The border is of width `Gr+Tr` in the range dimension and `Gd+Td` in the Doppler dimension.


### Glossary

**CFAR**: Constant False Alarm Rate

**CA**: Cell Averaging

**OS**: Ordered Statistics

**MAMIS**: Maximum Minimum Statistics
