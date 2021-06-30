#include "hdf5_custom.h"
#include <hdf5.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "ads.h"


#define FILENAME    "filtered.h5"
#define DATASETNAME "ExtendibleArray"
#define RANK         2


hsize_t      maxdims[2] = {H5S_UNLIMITED, H5S_UNLIMITED};
herr_t       status;

HDF5Class::HDF5Class() {}
HDF5Class::~HDF5Class() {}

void HDF5Class::createHDF5File (hdf5Struct* hdf5data)
{
    printf("Creating HDF5 File\n");
    hdf5data->dims[0] = 1;
    hdf5data->dims[1] = 12;
    hsize_t      chunk_dims[2] = {1000, 12};
    double       data[1][12] = { {0.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0}};    /* data to write */

    /* Create the data space with unlimited dimensions. */
    hdf5data->dataspace = H5Screate_simple (RANK, hdf5data->dims, maxdims);

    /* Create a new file. If file exists its contents will be overwritten. */
    hdf5data->file = H5Fcreate (FILENAME, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    /* Modify dataset creation properties, i.e. enable chunking  */
    hdf5data->prop = H5Pcreate (H5P_DATASET_CREATE);
    status = H5Pset_chunk (hdf5data->prop, RANK, chunk_dims);

    /* Create a new dataset within the file using chunk
       creation properties.  */
    hdf5data->dataset = H5Dcreate2 (hdf5data->file, DATASETNAME, H5T_IEEE_F64BE, hdf5data->dataspace,
                         H5P_DEFAULT, hdf5data->prop, H5P_DEFAULT);

    /* Write data to dataset */
    status = H5Dwrite (hdf5data->dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL,
                       H5P_DEFAULT, data);

    /* ATTRIBUTES */
//    hid_t attr1,attr2,attr3; //Attribute identifier
//    hid_t atype; //Attribute type
//    herr_t ret; // Return val

//    /* Create dataspace for the first attribute */
//    hdf5data->aid3 = H5Screate(H5S_SCALAR);
//    atype = H5Tcopy(H5T_C_S1);

//    /* DATA TO WRITE */
//    //channel setup
////    std::string stringChannelSetup = "CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,TEMP,YAW,PITCH,ROLL";
////    char charChannelSetup[stringChannelSetup.size()+1];
////    strcpy(charChannelSetup,stringChannelSetup.c_str());
//    char stringChannel[] ="CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8,TEMP,YAW,PITCH,ROLL";
//    //sampling rate
////    std::string stringSamplingRate = std::to_string(samplingRate);
////    char charSamplingRate[stringSamplingRate.size()+1];
////    strcpy(charSamplingRate,stringSamplingRate.c_str());

//    //Set length and name of attribute
//    H5Tset_size(atype,50);
//    attr1 = H5Acreate(hdf5data->dataset,"CHANNEL SETUP",atype,hdf5data->aid3,H5P_DEFAULT,H5P_DEFAULT);
////    H5Tset_size(atype,stringSamplingRate.size()+1);
////    attr2 = H5Acreate(hdf5data->dataset,"SAMPLING RATE",atype,hdf5data->aid3,H5P_DEFAULT,H5P_DEFAULT);
//    //attr3 = H5Acreate(hdf5data->dataset,"CHAR ATTRIBUTE3",atype,hdf5data->aid3,H5P_DEFAULT,H5P_DEFAULT);



//    /* Write string attribute */
//    ret = H5Awrite(attr1,atype,stringChannel);
////    ret = H5Awrite(attr2,atype,charSamplingRate);
//    //ret = H5Awrite(attr3,atype,stringAttr3);


}

void HDF5Class::writeToHDF5 (hdf5Struct* hdf5data, double **RESULTARRAY, int blockSize, unsigned long int counter, int arrayOffset)
{
    //qDebug() << "counter:" << counter << "blockSize:" << blockSize << "offset:" << arrayOffset;
    //printf("Write to HDF5\n");
    // CREATE COPY OF ARRAY TO SAVE
    double tempArray[blockSize][12];
    for (int i=0;i<blockSize;i++) {
        //if(i==0) qDebug() << RESULTARRAY[i+arrayOffset][0];
        for (int j=0;j<12;j++) {
            tempArray[i][j] = RESULTARRAY[i+arrayOffset][j];
        }
    }
    int rank;

    //hdf5data->file = H5Fopen (FILENAME, H5F_ACC_RDONLY, H5P_DEFAULT);
    //hdf5data->dataset = H5Dopen2 (hdf5data->file, DATASETNAME, H5P_DEFAULT);

    hsize_t      dimsext[2] = {blockSize, 12};         /* extend dimensions */

    /* Extend the dataset. Dataset becomes 100 * x  */
    hdf5data->size[0] = counter; //* (blockUeberlauf+1) ;
    hdf5data->size[1] = hdf5data->dims[1];
    status = H5Dset_extent (hdf5data->dataset, hdf5data->size);


    /* Select a hyperslab in extended portion of dataset  */
    hdf5data->filespace = H5Dget_space (hdf5data->dataset);
    rank = H5Sget_simple_extent_ndims (hdf5data->filespace);

    hdf5data->offset[0] = counter - blockSize;
    hdf5data->offset[1] = 0;
    status = H5Sselect_hyperslab (hdf5data->filespace, H5S_SELECT_SET, hdf5data->offset, NULL,
                                  dimsext, NULL);

    /* Define memory space */
    hdf5data->memspace = H5Screate_simple (rank, dimsext, NULL);

    /* Write the data to the extended portion of dataset  */
    status = H5Dwrite (hdf5data->dataset, H5T_NATIVE_DOUBLE, hdf5data->memspace, hdf5data->filespace,
                       H5P_DEFAULT, tempArray);


}

void HDF5Class::closeHDF5File(hdf5Struct* hdf5data) {
    printf("Closeing HDF5\n");

    //status = H5Sclose (hdf5data->aid3);
    status = H5Dclose (hdf5data->dataset);
    status = H5Pclose (hdf5data->prop);
    status = H5Sclose (hdf5data->dataspace);
    status = H5Sclose (hdf5data->memspace);
    status = H5Sclose (hdf5data->filespace);
    status = H5Fclose (hdf5data->file);
}

