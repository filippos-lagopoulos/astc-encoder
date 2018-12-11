#include <stdint.h>

struct astc_codec_image;
namespace astclib
{
enum ASTC_COMPRESS_MODE     // Trade-off compression quality for speed
{
	ASTC_COMPRESS_VERY_FAST,
	ASTC_COMPRESS_FAST,
	ASTC_COMPRESS_MEDIUM,
	ASTC_COMPRESS_THOROUGH,
	ASTC_COMPRESS_EXHAUSTIVE,
};
class ASTCCompressor
{
public:
    ASTCCompressor();
    virtual ~ASTCCompressor();

    void compressRawRGBA(
        ASTC_COMPRESS_MODE mode,
        int threadCount,
        int xdim,
        int ydim,
        uint8_t* inputBuffer,
        uint8_t* outputBuffer,
        int inputWidth,
        int inputHeight);

private:
    void copyRawData(uint8_t* rawData, int width, int height);

private:
    astc_codec_image* m_astc_image = nullptr;
    uint8_t* m_outBuffer = nullptr;
};
}