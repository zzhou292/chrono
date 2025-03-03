// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include <iostream>
#include <sstream>
#include <math.h>
#include <set>
#include "mpi.h"

#include "chrono_multidomain/ChMpi.h"

namespace chrono {
namespace multidomain {

using namespace std;

ChMPIrequest::ChMPIrequest() {
    this->mpireq = new MPI_Request;
}

ChMPIrequest::~ChMPIrequest() {
    MPI_Request* mr = static_cast<MPI_Request*>(this->mpireq);
    delete (mr);
    this->mpireq = 0;
}

ChMPIstatus::ChMPIstatus() {
    this->mpistat = new MPI_Status;
}

ChMPIstatus::~ChMPIstatus() {
    MPI_Status* mr = static_cast<MPI_Status*>(this->mpistat);
    delete (mr);
    this->mpistat = 0;
}

int ChMPI::Init(int* argc, char** argv[]) {
    return MPI_Init(argc, argv);
}

int ChMPI::Finalize() {
    return MPI_Finalize();
}

int ChMPI::CommSize() {
    int numprocs = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
    return numprocs;
}

int ChMPI::CommRank() {
    int myid;
    MPI_Comm_rank(MPI_COMM_WORLD, &myid);
    return myid;
}

int ChMPI::Wait(ChMPIrequest* mreq, ChMPIstatus* mstatus) {
    return MPI_Wait((MPI_Request*)mreq->mpireq, (MPI_Status*)mstatus->mpistat);
}

bool ChMPI::Test(ChMPIrequest* mreq, ChMPIstatus* mstatus) {
    int flag;
    MPI_Test((MPI_Request*)mreq->mpireq, &flag, (MPI_Status*)mstatus->mpistat);
    return (flag != 0);
}

int ChMPI::SendMatrix_blocking(int destID, ChMatrixDynamic<double>& source_matr, eCh_mpiCommMode mmode) {
    struct Matrixstruct {
        int rows;
        int cols;
        double* vals;
    };
    Matrixstruct mmatr;
    mmatr.rows = source_matr.rows();
    mmatr.cols = source_matr.rows();
    mmatr.vals = source_matr.data();

    MPI_Datatype Matrixtype;
    MPI_Datatype type[3] = {MPI_INT, MPI_INT, MPI_DOUBLE};
    int blocklen[3] = {1, 1, mmatr.rows * mmatr.cols};
    MPI_Aint disp[3];
    MPI_Aint base;
    // compute displacements of structure components
    MPI_Get_address(&mmatr.rows, disp);
    MPI_Get_address(&mmatr.cols, disp + 1);
    MPI_Get_address(mmatr.vals, disp + 2);
    base = disp[0];
    for (int i = 0; i < 3; i++)
        disp[i] -= base;
    MPI_Type_create_struct(3, blocklen, disp, type, &Matrixtype);
    MPI_Type_commit(&Matrixtype);

    int err = 0;

    switch (mmode) {
        case ChMPI::MPI_STANDARD:
            err = MPI_Send(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
            break;
        case ChMPI::MPI_BUFFERED:
            err = MPI_Bsend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
            break;
        case ChMPI::MPI_SYNCHRONOUS:
            err = MPI_Ssend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
            break;
        case ChMPI::MPI_READY:
            err = MPI_Rsend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD);
            break;
        default:
            break;
    }

    MPI_Type_free(&Matrixtype);

    return err;
}

int ChMPI::SendMatrix_nonblocking(int destID,
                                  ChMatrixDynamic<double>& source_matr,
                                  eCh_mpiCommMode mmode,
                                  ChMPIrequest* mreq) {
    struct Matrixstruct {
        int rows;
        int cols;
        double* vals;
    };
    Matrixstruct mmatr;
    mmatr.rows = source_matr.rows();
    mmatr.cols = source_matr.rows();
    mmatr.vals = source_matr.data();

    MPI_Datatype Matrixtype;
    MPI_Datatype type[3] = {MPI_INT, MPI_INT, MPI_DOUBLE};
    int blocklen[3] = {1, 1, mmatr.rows * mmatr.cols};
    MPI_Aint disp[3];
    MPI_Aint base;
    // compute displacements of structure components
    MPI_Get_address(&mmatr.rows, disp);
    MPI_Get_address(&mmatr.cols, disp + 1);
    MPI_Get_address(mmatr.vals, disp + 2);
    base = disp[0];
    for (int i = 0; i < 3; i++)
        disp[i] -= base;
    MPI_Type_create_struct(3, blocklen, disp, type, &Matrixtype);
    MPI_Type_commit(&Matrixtype);

    int err = 0;

    switch (mmode) {
        case ChMPI::MPI_STANDARD:
            err = MPI_Isend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        case ChMPI::MPI_BUFFERED:
            err = MPI_Ibsend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        case ChMPI::MPI_SYNCHRONOUS:
            err = MPI_Issend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        case ChMPI::MPI_READY:
            err = MPI_Irsend(&mmatr, 1, Matrixtype, destID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        default:
            break;
    }

    MPI_Type_free(&Matrixtype);

    return err;
}

int ChMPI::ReceiveMatrix_blocking(int sourceID, ChMatrixDynamic<double>& dest_matr, ChMPIstatus* mstatus) {
    struct Matrixstruct {
        int rows;
        int cols;
        double* vals;
    };
    Matrixstruct mmatr;
    // mmatr.rows = dest_matr.GetRows();
    // mmatr.cols = dest_matr.GetColumns();
    mmatr.vals = dest_matr.data();
    int nelements = dest_matr.rows() * dest_matr.cols();

    MPI_Datatype Matrixtype;
    MPI_Datatype type[3] = {MPI_INT, MPI_INT, MPI_DOUBLE};
    int blocklen[3] = {1, 1, nelements};
    MPI_Aint disp[3];
    MPI_Aint base;
    // compute displacements of structure components
    MPI_Get_address(&mmatr.rows, disp);
    MPI_Get_address(&mmatr.cols, disp + 1);
    MPI_Get_address(mmatr.vals, disp + 2);
    base = disp[0];
    for (int i = 0; i < 3; i++)
        disp[i] -= base;
    MPI_Type_create_struct(3, blocklen, disp, type, &Matrixtype);
    MPI_Type_commit(&Matrixtype);

    int err = MPI_Recv(&mmatr, 1, Matrixtype, sourceID, 1001, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);

    // int nnelements;
    // MPI_Get_count ( (MPI_Status*)mstatus->mpistat, Matrixtype, &nnelements );
    // GetLog() << " n.elements:" << nnelements << "\n";

    MPI_Type_free(&Matrixtype);

    return err;
}

int ChMPI::ReceiveMatrix_nonblocking(int sourceID, ChMatrixDynamic<double>& dest_matr, ChMPIrequest* mreq) {
    struct Matrixstruct {
        int rows;
        int cols;
        double* vals;
    };
    Matrixstruct mmatr;
    // mmatr.rows = dest_matr.GetRows();
    // mmatr.cols = dest_matr.GetColumns();
    mmatr.vals = dest_matr.data();
    int nelements = dest_matr.rows() * dest_matr.cols();

    MPI_Datatype Matrixtype;
    MPI_Datatype type[3] = {MPI_INT, MPI_INT, MPI_DOUBLE};
    int blocklen[3] = {1, 1, nelements};
    MPI_Aint disp[3];
    MPI_Aint base;
    // compute displacements of structure components
    MPI_Get_address(&mmatr.rows, disp);
    MPI_Get_address(&mmatr.cols, disp + 1);
    MPI_Get_address(mmatr.vals, disp + 2);
    base = disp[0];
    for (int i = 0; i < 3; i++)
        disp[i] -= base;
    MPI_Type_create_struct(3, blocklen, disp, type, &Matrixtype);
    MPI_Type_commit(&Matrixtype);

    int err = MPI_Irecv(&mmatr, 1, Matrixtype, sourceID, 1001, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);

    // int nnelements;
    // MPI_Get_count ( (MPI_Status*)mstatus->mpistat, Matrixtype, &nnelements );
    // GetLog() << " n.elements:" << nnelements << "\n";

    MPI_Type_free(&Matrixtype);

    return err;
}

int ChMPI::SendString_blocking(int destID,               ///< destination rank
                               std::string& source_str,  ///< source string
                               eCh_mpiCommMode mmode     ///< send mode
) {
    char* data = (char*)source_str.data();  // should not access directly std::string data, but this is efficient!
    int nbytes = (int)source_str.size();
    int err = 0;

    switch (mmode) {
        case ChMPI::MPI_STANDARD:
            err = MPI_Send(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
            break;
        case ChMPI::MPI_BUFFERED:
            err = MPI_Bsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
            break;
        case ChMPI::MPI_SYNCHRONOUS:
            err = MPI_Ssend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
            break;
        case ChMPI::MPI_READY:
            err = MPI_Rsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD);
            break;
        default:
            break;
    }

    return err;
}

int ChMPI::SendString_nonblocking(int destID,               ///< destination rank
                                  std::string& source_str,  ///< source string
                                  eCh_mpiCommMode mmode,    ///< send mode
                                  ChMPIrequest* mreq        ///< if nonblocking=true, must use this
) {
    char* data = (char*)source_str.data();  // should not access directly std::string data, but this is efficient!
    int nbytes = (int)source_str.size();
    int err = 0;

    switch (mmode) {
        case ChMPI::MPI_STANDARD:
            err = MPI_Isend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        case ChMPI::MPI_BUFFERED:
            err = MPI_Ibsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        case ChMPI::MPI_SYNCHRONOUS:
            err = MPI_Issend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        case ChMPI::MPI_READY:
            err = MPI_Irsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
            break;
        default:
            break;
    }

    return err;
}

/*
int ChMPI::SendString_nonblocking2(
    int destID,					///< destination rank
    std::string& source_str,	///< source string
    eCh_mpiCommMode mmode,		///< send mode
    ChMPIrequest* mreq			///< if nonblocking=true, must use this
)
{
    char* data = (char*)source_str.data(); // should not access directly std::string data, but this is efficient!
    int nbytes = (int)source_str.size();
    int err = 0;

    switch (mmode)
    {
    case ChMPI::MPI_STANDARD:
        err = MPI_Isend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        err = MPI_Isend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        break;
    case ChMPI::MPI_BUFFERED:
        err = MPI_Ibsend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        err = MPI_Ibsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        break;
    case ChMPI::MPI_SYNCHRONOUS:
        err = MPI_Issend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        err = MPI_Issend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        break;
    case ChMPI::MPI_READY:
        err = MPI_Irsend(&nbytes, 1, MPI_INT, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        err = MPI_Irsend(data, nbytes, MPI_BYTE, destID, 1002, MPI_COMM_WORLD, (MPI_Request*)mreq->mpireq);
        break;
    default:
        break;
    }

    return err;
}
*/

int ChMPI::ReceiveString_blocking(int sourceID,           ///< source rank
                                  std::string& dest_str,  ///< destination string - will be resized
                                  ChMPIstatus* mstatus) {
    MPI_Status status;
    int incoming_msg_size = 0;
    MPI_Probe(sourceID, 1002, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_BYTE, &incoming_msg_size);
    // GetLog() << "   size:" << incoming_msg_size << "   \n";

    dest_str.resize(incoming_msg_size);

    void* data = (void*)dest_str.data();  // should not access directly std::string data! but this is efficient!

    int err = 0;
    if (incoming_msg_size) {
        err =
            MPI_Recv(data, incoming_msg_size, MPI_BYTE, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
    }
    return err;
}

/*
int ChMPI::ReceiveString_blocking2(
    int sourceID,				///< source rank
    std::string& dest_str,		///< destination string - will be resized
    ChMPIstatus* mstatus
) {
    int recv_length;
    MPI_Recv(&recv_length, 1, MPI_INT, sourceID, 1002, MPI_COMM_WORLD, (MPI_Status*)mstatus->mpistat);
    dest_str.resize(recv_length);
    int err = MPI_Recv(&dest_str[0], recv_length, MPI_BYTE, sourceID, 1002, MPI_COMM_WORLD,
(MPI_Status*)mstatus->mpistat); return err;
}
*/

int ChMPI::Barrier() {
    return MPI_Barrier(MPI_COMM_WORLD);
}

int ChMPI::WaitAll(int arraysize, ChMPIrequest requests[], ChMPIstatus statuses[]) {
    for (int i = 0; i < arraysize; ++i)
        int res = MPI_Wait((MPI_Request*)requests[i].mpireq, (MPI_Status*)statuses[i].mpistat);
    return 1;
}

int ChMPI::ReduceAll(double send, double& received_result, eCh_mpiReduceOperation operation) {
    MPI_Op mpi_operation = MPI_NO_OP;
    switch (operation) {
        case eCh_mpiReduceOperation::MPI_max:
            mpi_operation = MPI_MAX;
            break;
        case eCh_mpiReduceOperation::MPI_min:
            mpi_operation = MPI_MIN;
            break;
        case eCh_mpiReduceOperation::MPI_sum:
            mpi_operation = MPI_SUM;
            break;
        case eCh_mpiReduceOperation::MPI_prod:
            mpi_operation = MPI_PROD;
            break;
    }

    return MPI_Allreduce(&send, &received_result, 1, MPI_DOUBLE, mpi_operation, MPI_COMM_WORLD);
}

template <typename T>
void ChMPI::ChBroadcast(std::vector<T>& data, int root) {
    // First broadcast the size of the outer vector
    int size = data.size();
    MPI_Bcast(&size, 1, MPI_INT, root, MPI_COMM_WORLD);

    // Resize receiving buffers on non-root ranks
    if (GetRank() != root) {
        data.resize(size);
    }

    // For AABB type, we need special handling for the tags vector
    if constexpr (std::is_same_v<T, AABB>) {
        // Broadcast the fixed part of each AABB first
        for (int i = 0; i < size; i++) {
            MPI_Bcast(&data[i].min, 3, MPI_DOUBLE, root, MPI_COMM_WORLD);
            MPI_Bcast(&data[i].max, 3, MPI_DOUBLE, root, MPI_COMM_WORLD);

            // Broadcast size of tags vector for this AABB
            int tags_size = data[i].tags.size();
            MPI_Bcast(&tags_size, 1, MPI_INT, root, MPI_COMM_WORLD);

            // Resize and broadcast tags
            if (GetRank() != root) {
                data[i].tags.resize(tags_size);
            }
            MPI_Bcast(data[i].tags.data(), tags_size, MPI_INT, root, MPI_COMM_WORLD);
        }
    } else {
        // For other types, use the original byte-based broadcast
        MPI_Bcast(data.data(), size * sizeof(T), MPI_BYTE, root, MPI_COMM_WORLD);
    }
}

// Add a scalar broadcast function
template <typename T>
void ChMPI::ChBroadcast(T* value, int count, int root) {
    // For basic types, use MPI_Bcast with appropriate MPI datatype
    if constexpr (std::is_same_v<T, int>) {
        MPI_Bcast(value, count, MPI_INT, root, MPI_COMM_WORLD);
    } else if constexpr (std::is_same_v<T, double>) {
        MPI_Bcast(value, count, MPI_DOUBLE, root, MPI_COMM_WORLD);
    } else if constexpr (std::is_same_v<T, float>) {
        MPI_Bcast(value, count, MPI_FLOAT, root, MPI_COMM_WORLD);
    } else if constexpr (std::is_same_v<T, char>) {
        MPI_Bcast(value, count, MPI_CHAR, root, MPI_COMM_WORLD);
    } else if constexpr (std::is_same_v<T, bool>) {
        // MPI doesn't have a direct boolean type, so use MPI_BYTE
        MPI_Bcast(value, count, MPI_BYTE, root, MPI_COMM_WORLD);
    } else {
        // For other types, use byte-based broadcast
        MPI_Bcast(value, count * sizeof(T), MPI_BYTE, root, MPI_COMM_WORLD);
    }
}

// Explicit template instantiations for common types
template void ChMPI::ChBroadcast<int>(int* value, int count, int root);
template void ChMPI::ChBroadcast<double>(double* value, int count, int root);
template void ChMPI::ChBroadcast<float>(float* value, int count, int root);
template void ChMPI::ChBroadcast<char>(char* value, int count, int root);
template void ChMPI::ChBroadcast<bool>(bool* value, int count, int root);

int ChMPI::GetRank() {
    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    return rank;
}
////////////////////////////////////////////////////

double f(double);

double f(double a) {
    return (4.0 / (1.0 + a * a));
}

int ccmain(int argc, char* argv[]) {
    int done = 0, n, myid, numprocs, i;
    double PI25DT = 3.141592653589793238462643;
    double mypi, pi, h, sum, x;
    double startwtime = 0.0, endwtime;
    int namelen;
    char processor_name[MPI_MAX_PROCESSOR_NAME];

    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &numprocs);
    MPI_Comm_rank(MPI_COMM_WORLD, &myid);
    MPI_Get_processor_name(processor_name, &namelen);

    /*
    fprintf(stdout,"Process %d of %d is on %s\n",
        myid, numprocs, processor_name);
    fflush(stdout);
    */

    while (!done) {
        if (myid == 0) {
            fprintf(stdout, "Enter the number of intervals: (0 quits) ");
            fflush(stdout);
            if (scanf("%d", &n) != 1) {
                fprintf(stdout, "No number entered; quitting\n");
                n = 0;
            }
            startwtime = MPI_Wtime();
        }
        MPI_Bcast(&n, 1, MPI_INT, 0, MPI_COMM_WORLD);
        if (n == 0)
            done = 1;
        else {
            h = 1.0 / (double)n;
            sum = 0.0;
            for (i = myid + 1; i <= n; i += numprocs) {
                x = h * ((double)i - 0.5);
                sum += f(x);
            }
            mypi = h * sum;
            MPI_Reduce(&mypi, &pi, 1, MPI_DOUBLE, MPI_SUM, 0, MPI_COMM_WORLD);

            if (myid == 0) {
                printf("pi is approximately %.16f, Error is %.16f\n", pi, fabs(pi - PI25DT));
                endwtime = MPI_Wtime();
                printf("wall clock time = %f\n", endwtime - startwtime);
                fflush(stdout);
            }
        }
    }
    MPI_Finalize();
    return 0;
}

void Ch_test_mpi::run_test() {
    int foo = 0;
    char* faa = (char*)"";
    ccmain(foo, &faa);
}

template void ChMPI::ChBroadcast<AABB>(std::vector<AABB>& data, int root);

template <typename T>
void ChMPI::ChAllreduce(const std::vector<T>& sendbuf, std::vector<T>& recvbuf, ChOperation op) {
    recvbuf.resize(sendbuf.size());
    if constexpr (std::is_same_v<T, ChAABB>) {
        int rank, num_ranks;
        MPI_Comm_rank(MPI_COMM_WORLD, &rank);
        MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

        // First, copy the send buffer to receive buffer
        recvbuf = sendbuf;

        // For each AABB in the vector
        for (size_t i = 0; i < sendbuf.size(); i++) {
            // Create temporary arrays for min/max coordinates
            double min_coords[3], max_coords[3];

            // Each rank contributes its own data for position i
            if (i == rank) {
                min_coords[0] = sendbuf[i].min.x();
                min_coords[1] = sendbuf[i].min.y();
                min_coords[2] = sendbuf[i].min.z();
                max_coords[0] = sendbuf[i].max.x();
                max_coords[1] = sendbuf[i].max.y();
                max_coords[2] = sendbuf[i].max.z();
            } else {
                // Initialize with neutral values for the operation
                min_coords[0] = min_coords[1] = min_coords[2] = DBL_MAX;   // For MIN op
                max_coords[0] = max_coords[1] = max_coords[2] = -DBL_MAX;  // For MAX op
            }

            // Temporary arrays to receive the reduced values
            double min_result[3], max_result[3];

            // Use MPI_Allreduce to combine values from all ranks
            MPI_Allreduce(min_coords, min_result, 3, MPI_DOUBLE, MPI_MIN, MPI_COMM_WORLD);
            MPI_Allreduce(max_coords, max_result, 3, MPI_DOUBLE, MPI_MAX, MPI_COMM_WORLD);

            // Update the receive buffer with the reduced values
            recvbuf[i].min = ChVector3d(min_result[0], min_result[1], min_result[2]);
            recvbuf[i].max = ChVector3d(max_result[0], max_result[1], max_result[2]);
        }
    } else {
        // Handle non-AABB types as before...
        MPI_Op mpi_op;

        switch (op) {
            case ChOperation::MAX:
                mpi_op = MPI_MAX;
                break;
            case ChOperation::MIN:
                mpi_op = MPI_MIN;
                break;
            case ChOperation::SUM:
                mpi_op = MPI_SUM;
                break;
            case ChOperation::PROD:
                mpi_op = MPI_PROD;
                break;
            default:
                mpi_op = MPI_SUM;
                break;
        }

        MPI_Allreduce(sendbuf.data(), recvbuf.data(), sendbuf.size() * sizeof(T), MPI_BYTE, mpi_op, MPI_COMM_WORLD);
    }
}

// Add explicit template instantiation for ChAABB
template void ChMPI::ChAllreduce<ChAABB>(const std::vector<ChAABB>& sendbuf,
                                         std::vector<ChAABB>& recvbuf,
                                         ChOperation op);

template <typename T>
void ChMPI::GatherToMaster(const std::vector<T>& sendbuf, std::vector<T>& recvbuf, int master_rank) {
    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    int num_ranks;
    MPI_Comm_size(MPI_COMM_WORLD, &num_ranks);

    // First, broadcast the size of each rank's data
    int local_size = sendbuf.size();
    std::vector<int> sizes(num_ranks);

    // Each rank broadcasts its size to all others
    for (int i = 0; i < num_ranks; i++) {
        int size_to_broadcast = (i == rank) ? local_size : 0;
        MPI_Bcast(&size_to_broadcast, 1, MPI_INT, i, MPI_COMM_WORLD);
        sizes[i] = size_to_broadcast;
    }

    // Master collects all data
    if (rank == master_rank) {
        // Pre-allocate space for all data
        int total_size = 0;
        for (int size : sizes) {
            total_size += size;
        }
        recvbuf.resize(total_size);

        // Master adds its own data first
        int current_index = 0;
        for (const auto& item : sendbuf) {
            recvbuf[current_index++] = item;
        }

        // Receive data from other ranks
        if constexpr (std::is_same_v<T, AABB>) {
            // Special handling for AABB type
            for (int i = 0; i < num_ranks; i++) {
                if (i != master_rank && sizes[i] > 0) {
                    // Receive AABBs from rank i
                    for (int j = 0; j < sizes[i]; j++) {
                        // For each AABB, receive min and max
                        ChVector3d min, max;
                        MPI_Recv(&min, 3, MPI_DOUBLE, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                        MPI_Recv(&max, 3, MPI_DOUBLE, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

                        // Receive tags
                        int tags_size;
                        MPI_Recv(&tags_size, 1, MPI_INT, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                        std::vector<int> tags(tags_size);
                        MPI_Recv(tags.data(), tags_size, MPI_INT, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);

                        // Create AABB and add to recvbuf
                        recvbuf[current_index++] = AABB(min, max, tags);
                    }
                }
            }
        } else {
            // For other types, use a simpler approach
            for (int i = 0; i < num_ranks; i++) {
                if (i != master_rank && sizes[i] > 0) {
                    std::vector<T> received_data(sizes[i]);

                    // Receive data from rank i
                    for (int j = 0; j < sizes[i]; j++) {
                        MPI_Recv(&received_data[j], sizeof(T), MPI_BYTE, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                    }

                    // Add received data to recvbuf
                    for (const auto& item : received_data) {
                        recvbuf[current_index++] = item;
                    }
                }
            }
        }
    } else {
        // Non-master ranks send their data to master
        if constexpr (std::is_same_v<T, AABB>) {
            // Special handling for AABB type
            for (const auto& aabb : sendbuf) {
                // Send min and max
                ChVector3d min(aabb.min[0], aabb.min[1], aabb.min[2]);
                ChVector3d max(aabb.max[0], aabb.max[1], aabb.max[2]);
                MPI_Send(&min, 3, MPI_DOUBLE, master_rank, 0, MPI_COMM_WORLD);
                MPI_Send(&max, 3, MPI_DOUBLE, master_rank, 0, MPI_COMM_WORLD);

                // Send tags
                int tags_size = aabb.tags.size();
                MPI_Send(&tags_size, 1, MPI_INT, master_rank, 0, MPI_COMM_WORLD);
                MPI_Send(aabb.tags.data(), tags_size, MPI_INT, master_rank, 0, MPI_COMM_WORLD);
            }
        } else {
            // For other types, use a simpler approach
            for (const auto& item : sendbuf) {
                MPI_Send(&item, sizeof(T), MPI_BYTE, master_rank, 0, MPI_COMM_WORLD);
            }
        }
    }

    // barrier
    MPI_Barrier(MPI_COMM_WORLD);
}

// Explicit template instantiation for AABB
template void ChMPI::GatherToMaster<AABB>(const std::vector<AABB>& sendbuf,
                                          std::vector<AABB>& recvbuf,
                                          int master_rank);

}  // end namespace multidomain
}  // end namespace chrono
