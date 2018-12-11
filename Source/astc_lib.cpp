
#include "astc_lib.h"
#include "astc_codec_internals.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>

extern int block_mode_histogram[2048];
int rgb_force_use_of_hdr = 0;
int alpha_force_use_of_hdr = 0;
int perform_srgb_transform = 0;
int print_tile_errors = 0;
int print_statistics = 0;
#ifdef DEBUG_PRINT_DIAGNOSTICS
	int print_diagnostics = 0;
	int diagnostics_tile = -1;
#endif

int astc_codec_unlink(const char *filename)
{
    return unlink(filename);
}

void astc_codec_internal_error(const char *filename, int linenum)
{
	printf("Internal error: File=%s Line=%d\n", filename, linenum);
	exit(1);
}

namespace astclib
{
struct encode_astc_image_info
{
	int xdim;
	int ydim;
	int zdim;
	const error_weighting_params *ewp;
	uint8_t *buffer;
	int *counters;
	int pack_and_unpack;
	int thread_id;
	int threadcount;
	astc_decode_mode decode_mode;
	swizzlepattern swz_encode;
	swizzlepattern swz_decode;
	int *threads_completed;
	const astc_codec_image *input_image;
	astc_codec_image *output_image;
};

void *encode_astc_image_threadfunc(void *vblk)
{
	const encode_astc_image_info *blk = (const encode_astc_image_info *)vblk;
	int xdim = blk->xdim;
	int ydim = blk->ydim;
	int zdim = blk->zdim;
	uint8_t *buffer = blk->buffer;
	const error_weighting_params *ewp = blk->ewp;
	int thread_id = blk->thread_id;
	int threadcount = blk->threadcount;
	int *counters = blk->counters;
	int pack_and_unpack = blk->pack_and_unpack;
	astc_decode_mode decode_mode = blk->decode_mode;
	swizzlepattern swz_encode = blk->swz_encode;
	swizzlepattern swz_decode = blk->swz_decode;
	int *threads_completed = blk->threads_completed;
	const astc_codec_image *input_image = blk->input_image;
	astc_codec_image *output_image = blk->output_image;

	imageblock pb;
	int ctr = thread_id;
	int pctr = 0;

	int x, y, z, i;
	int xsize = input_image->xsize;
	int ysize = input_image->ysize;
	int zsize = input_image->zsize;
	int xblocks = (xsize + xdim - 1) / xdim;
	int yblocks = (ysize + ydim - 1) / ydim;
	int zblocks = (zsize + zdim - 1) / zdim;

	//allocate memory for temporary buffers
	compress_symbolic_block_buffers temp_buffers;
	temp_buffers.ewb = new error_weight_block;
	temp_buffers.ewbo = new error_weight_block_orig;
	temp_buffers.tempblocks = new symbolic_compressed_block[4];
	temp_buffers.temp = new imageblock;
	temp_buffers.planes2 = new compress_fixed_partition_buffers;
	temp_buffers.planes2->ei1 = new endpoints_and_weights;
	temp_buffers.planes2->ei2 = new endpoints_and_weights;
	temp_buffers.planes2->eix1 = new endpoints_and_weights[MAX_DECIMATION_MODES];
	temp_buffers.planes2->eix2 = new endpoints_and_weights[MAX_DECIMATION_MODES];
	temp_buffers.planes2->decimated_quantized_weights = new float[2 * MAX_DECIMATION_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.planes2->decimated_weights = new float[2 * MAX_DECIMATION_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.planes2->flt_quantized_decimated_quantized_weights = new float[2 * MAX_WEIGHT_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.planes2->u8_quantized_decimated_quantized_weights = new uint8_t[2 * MAX_WEIGHT_MODES * MAX_WEIGHTS_PER_BLOCK];
	temp_buffers.plane1 = temp_buffers.planes2;

	for (z = 0; z < zblocks; z++)
		for (y = 0; y < yblocks; y++)
			for (x = 0; x < xblocks; x++)
			{
				if (ctr == 0)
				{
					int offset = ((z * yblocks + y) * xblocks + x) * 16;
					uint8_t *bp = buffer + offset;

                    fetch_imageblock(input_image, &pb, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, swz_encode);
                    symbolic_compressed_block scb;
                    compress_symbolic_block(input_image, decode_mode, xdim, ydim, zdim, ewp, &pb, &scb, &temp_buffers);
                    if (pack_and_unpack)
                    {
                        decompress_symbolic_block(decode_mode, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, &scb, &pb);
                        write_imageblock(output_image, &pb, xdim, ydim, zdim, x * xdim, y * ydim, z * zdim, swz_decode);
                    }
                    else
                    {
                        physical_compressed_block pcb;
                        pcb = symbolic_to_physical(xdim, ydim, zdim, &scb);
                        *(physical_compressed_block *) bp = pcb;
                    }

					counters[thread_id]++;
					ctr = threadcount - 1;

					pctr++;
				}
				else
					ctr--;
			}

	delete[] temp_buffers.planes2->decimated_quantized_weights;
	delete[] temp_buffers.planes2->decimated_weights;
	delete[] temp_buffers.planes2->flt_quantized_decimated_quantized_weights;
	delete[] temp_buffers.planes2->u8_quantized_decimated_quantized_weights;
	delete[] temp_buffers.planes2->eix1;
	delete[] temp_buffers.planes2->eix2;
	delete   temp_buffers.planes2->ei1;
	delete   temp_buffers.planes2->ei2;
	delete   temp_buffers.planes2;
	delete[] temp_buffers.tempblocks;
	delete   temp_buffers.temp;
	delete   temp_buffers.ewbo;
	delete   temp_buffers.ewb;

	threads_completed[thread_id] = 1;
	return NULL;
}


void encode_astc_image(const astc_codec_image * input_image,
					   astc_codec_image * output_image,
					   int xdim,
					   int ydim,
					   int zdim,
					   error_weighting_params * ewp, astc_decode_mode decode_mode, swizzlepattern swz_encode, swizzlepattern swz_decode, uint8_t * buffer, int pack_and_unpack, int threadcount)
{
	int i;
	int *counters = new int[threadcount];
	int *threads_completed = new int[threadcount];

	// before entering into the multi-threaded routine, ensure that the block size descriptors
	// and the partition table descriptors needed actually exist.
	get_block_size_descriptor(xdim, ydim, zdim);

	get_partition_table(xdim, ydim, zdim, 0);

	encode_astc_image_info *ai = new encode_astc_image_info[threadcount];

	for (i = 0; i < threadcount; i++)
	{
		ai[i].xdim = xdim;
		ai[i].ydim = ydim;
		ai[i].zdim = zdim;
		ai[i].buffer = buffer;
		ai[i].ewp = ewp;
		ai[i].counters = counters;
		ai[i].pack_and_unpack = pack_and_unpack;
		ai[i].thread_id = i;
		ai[i].threadcount = threadcount;
		ai[i].decode_mode = decode_mode;
		ai[i].swz_encode = swz_encode;
		ai[i].swz_decode = swz_decode;
		ai[i].threads_completed = threads_completed;
		ai[i].input_image = input_image;
		ai[i].output_image = output_image;
		counters[i] = 0;
		threads_completed[i] = 0;
	}

	if (threadcount == 1)
		encode_astc_image_threadfunc(&ai[0]);
	else
	{
		pthread_t *threads = new pthread_t[threadcount];
		for (i = 0; i < threadcount; i++)
			pthread_create(&(threads[i]), NULL, encode_astc_image_threadfunc, (void *)(&(ai[i])));

		for (i = 0; i < threadcount; i++)
			pthread_join(threads[i], NULL);
		delete[]threads;
	}

	delete[]ai;
	delete[]counters;
	delete[]threads_completed;
}

astc_codec_image *load_uncompressed_image(int padding, int *result, int width, int height, unsigned char* pixels)
{
	astc_codec_image *astc_img = NULL;
	astc_img = allocate_image(8, width, height, 1, padding);
	bool y_flip = false;

	for (int y = 0; y < height; y++)
	{
		int y_dst = y + padding;
		int y_src = y_flip ? (height - y - 1) : y;
		uint8_t *src = pixels + 4 * width * y_src;

		for (int x = 0; x < width; x++)
		{
			int x_dst = x + padding;
			astc_img->imagedata8[0][y_dst][4 * x_dst] = src[4 * x];
			astc_img->imagedata8[0][y_dst][4 * x_dst + 1] = src[4 * x + 1];
			astc_img->imagedata8[0][y_dst][4 * x_dst + 2] = src[4 * x + 2];
			astc_img->imagedata8[0][y_dst][4 * x_dst + 3] = src[4 * x + 3];
		}
	}
	fill_image_padding_area(astc_img);
	return astc_img;
}


void init_ewp(error_weighting_params& ewp)
{
	ewp.rgb_power                   = 1.0f;
	ewp.alpha_power                 = 1.0f;
	ewp.rgb_base_weight             = 1.0f;
	ewp.alpha_base_weight           = 1.0f;
	ewp.rgb_mean_weight             = 0.0f;
	ewp.rgb_stdev_weight            = 0.0f;
	ewp.alpha_mean_weight           = 0.0f;
	ewp.alpha_stdev_weight          = 0.0f;

	ewp.rgb_mean_and_stdev_mixing   = 0.0f;
	ewp.mean_stdev_radius           = 0;
	ewp.enable_rgb_scale_with_alpha = 0;
	ewp.alpha_radius                = 0;

	ewp.block_artifact_suppression  = 0.0f;
	ewp.rgba_weights[0]             = 1.0f;
	ewp.rgba_weights[1]             = 1.0f;
	ewp.rgba_weights[2]             = 1.0f;
	ewp.rgba_weights[3]             = 1.0f;
	ewp.ra_normal_angular_scale     = 0;
}

void setup_ewp(ASTC_COMPRESS_MODE mode, int ydim, int xdim, error_weighting_params& ewp)
{
	float oplimit_autoset    = 0.0;
	float dblimit_autoset_2d = 0.0;
	float bmc_autoset        = 0.0;
	float mincorrel_autoset  = 0.0;

	int plimit_autoset       = -1;
	int maxiters_autoset     = 0;
	int pcdiv                = 1;

	float log10_texels_2d = log((float)(xdim * ydim)) / log(10.0f);

	if (mode == ASTC_COMPRESS_VERY_FAST)
	{
		plimit_autoset = 2;
		oplimit_autoset = 1.0;
		dblimit_autoset_2d = MAX(70 - 35 * log10_texels_2d, 53 - 19 * log10_texels_2d);
		bmc_autoset = 25;
		mincorrel_autoset = 0.5;
		maxiters_autoset = 1;

		switch (ydim)
		{
		case 4:
			pcdiv = 240;
			break;
		case 5:
			pcdiv = 56;
			break;
		case 6:
			pcdiv = 64;
			break;
		case 8:
			pcdiv = 47;
			break;
		case 10:
			pcdiv = 36;
			break;
		case 12:
			pcdiv = 30;
			break;
		default:
			pcdiv = 30;
			break;
		}
	}
	else if (mode == ASTC_COMPRESS_FAST)
	{
		plimit_autoset = 4;
		oplimit_autoset = 1.0;
		mincorrel_autoset = 0.5;
		dblimit_autoset_2d = MAX(85 - 35 * log10_texels_2d, 63 - 19 * log10_texels_2d);
		bmc_autoset = 50;
		maxiters_autoset = 1;

		switch (ydim)
		{
		case 4:
			pcdiv = 60;
			break;
		case 5:
			pcdiv = 27;
			break;
		case 6:
			pcdiv = 30;
			break;
		case 8:
			pcdiv = 24;
			break;
		case 10:
			pcdiv = 16;
			break;
		case 12:
			pcdiv = 20;
			break;
		default:
			pcdiv = 20;
			break;
		};
	}
	else if (mode == ASTC_COMPRESS_MEDIUM)
	{
		plimit_autoset = 25;
		oplimit_autoset = 1.2f;
		mincorrel_autoset = 0.75f;
		dblimit_autoset_2d = MAX(95 - 35 * log10_texels_2d, 70 - 19 * log10_texels_2d);
		bmc_autoset = 75;
		maxiters_autoset = 2;

		switch (ydim)
		{
		case 4:
			pcdiv = 25;
			break;
		case 5:
			pcdiv = 15;
			break;
		case 6:
			pcdiv = 15;
			break;
		case 8:
			pcdiv = 10;
			break;
		case 10:
			pcdiv = 8;
			break;
		case 12:
			pcdiv = 6;
			break;
		default:
			pcdiv = 6;
			break;
		};
	}
	else if (mode == ASTC_COMPRESS_THOROUGH)
	{
		plimit_autoset = 100;
		oplimit_autoset = 2.5f;
		mincorrel_autoset = 0.95f;
		dblimit_autoset_2d = MAX(105 - 35 * log10_texels_2d, 77 - 19 * log10_texels_2d);
		bmc_autoset = 95;
		maxiters_autoset = 4;

		switch (ydim)
		{
		case 4:
			pcdiv = 12;
			break;
		case 5:
			pcdiv = 7;
			break;
		case 6:
			pcdiv = 7;
			break;
		case 8:
			pcdiv = 5;
			break;
		case 10:
			pcdiv = 4;
			break;
		case 12:
			pcdiv = 3;
			break;
		default:
			pcdiv = 3;
			break;
		};
	}
	else if (mode == ASTC_COMPRESS_EXHAUSTIVE)
	{
		plimit_autoset = PARTITION_COUNT;
		oplimit_autoset = 1000.0f;
		mincorrel_autoset = 0.99f;
		dblimit_autoset_2d = 999.0f;
		bmc_autoset = 100;
		maxiters_autoset = 4;

		switch (ydim)
		{
		case 4:
			pcdiv = 3;
			break;
		case 5:
			pcdiv = 1;
			break;
		case 6:
			pcdiv = 1;
			break;
		case 8:
			pcdiv = 1;
			break;
		case 10:
			pcdiv = 1;
			break;
		case 12:
			pcdiv = 1;
			break;
		default:
			pcdiv = 1;
			break;
		}
	}

	int partitions_to_test = plimit_autoset;
	float dblimit_2d = dblimit_autoset_2d;
	float oplimit = oplimit_autoset;
	float mincorrel = mincorrel_autoset;

	int maxiters = maxiters_autoset;
	ewp.max_refinement_iters = maxiters;

	ewp.block_mode_cutoff = bmc_autoset / 100.0f;

	float texel_avg_error_limit_2d;

	if (rgb_force_use_of_hdr == 0)
	{
		texel_avg_error_limit_2d = pow(0.1f, dblimit_2d * 0.1f) * 65535.0f * 65535.0f;
	}
	else
	{
		texel_avg_error_limit_2d = 0.0f;
	}
	ewp.partition_1_to_2_limit = oplimit;
	ewp.lowest_correlation_cutoff = mincorrel;

	if (partitions_to_test < 1)
		partitions_to_test = 1;
	else if (partitions_to_test > PARTITION_COUNT)
		partitions_to_test = PARTITION_COUNT;
	ewp.partition_search_limit = partitions_to_test;

	ewp.texel_avg_error_limit = texel_avg_error_limit_2d;

	expand_block_artifact_suppression(xdim, ydim, 1, &ewp);
}

ASTCCompressor::ASTCCompressor()
{
    m_astc_image = allocate_image(8, 2048, 2048, 1, 0);
	// initialization routines
	prepare_angular_tables();
	build_quantization_mode_table();
}

ASTCCompressor::~ASTCCompressor()
{
    destroy_image(m_astc_image);
}

void ASTCCompressor::copyRawData(uint8_t* rawData, int width, int height)
{
    bool y_flip = false;
    int x, y = 0;
    for (y = 0; y < height; y++)
    {
        int y_dst = y;
        int y_src = y_flip ? (height - y - 1) : y;
        uint8_t *src = rawData + 4 * width * y_src;

        for (x = 0; x < width; x++)
        {
            m_astc_image->imagedata8[0][y_dst][4 * x] = src[4 * x];
            m_astc_image->imagedata8[0][y_dst][4 * x + 1] = src[4 * x + 1];
            m_astc_image->imagedata8[0][y_dst][4 * x + 2] = src[4 * x + 2];
            m_astc_image->imagedata8[0][y_dst][4 * x + 3] = src[4 * x + 3];
        }
    }
    m_astc_image->xsize = width;
    m_astc_image->ysize = height;
    m_astc_image->zsize = 1;
    m_astc_image->padding = 0;
}

void ASTCCompressor::compressRawRGBA(
	ASTC_COMPRESS_MODE mode,
	int threadCount,
	int xdim,
	int ydim,
	uint8_t* inputBuffer,
	uint8_t* outputBuffer,
	int inputWidth,
	int inputHeight)
{
    copyRawData(inputBuffer, inputWidth, inputHeight);

	error_weighting_params ewp;
	init_ewp(ewp);
	setup_ewp(mode, xdim, ydim, ewp);

  	static swizzlepattern swz_encode = { 0, 1, 2, 3 };

	encode_astc_image(m_astc_image, nullptr, xdim, ydim, 1, &ewp, DECODE_LDR, swz_encode, swz_encode, outputBuffer, 0, threadCount);
}
}