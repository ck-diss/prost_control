#ifndef HDF5_CUSTOM_H
#define HDF5_CUSTOM_H

#include <QTextStream>
#include <hdf5.h>
#include <qtconcurrentrun.h>

class HDF5Class {
   public:
        struct Hdf5Struct
        {
            hid_t        file;                          /* handles */
            hid_t        dataspace, dataset;
            hid_t        filespace, memspace;
            hid_t        prop;
            hsize_t      dims[2];
            /* Variables used in extending and writing to the extended portion of dataset */
            hsize_t      size[2];
            hsize_t      offset[2];
            hid_t aid3; //Attribute dataspace identifier

        };
        typedef struct Hdf5Struct hdf5Struct ;
        void createHDF5File(hdf5Struct *hdf5data);

        void closeHDF5File(hdf5Struct *hdf5data);
        HDF5Class();
        ~HDF5Class();
        void writeToHDF5(hdf5Struct *hdf5data,double**,int,unsigned long int,int);
        //void resultReady(const QString &result);

};


#endif // HDF5_CUSTOM_H
