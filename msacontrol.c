
#include <jni.h>  /* /usr/lib/jvm/java-1.7.0-openjdk-amd64/include/ */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "sensors_io.h"

#include <android/log.h>  /* liblog */

#define MSA_TAG "msa300_native"
 
#if 0
typedef struct {
    char *name;          /* Java里调用的函数名 */
    char *signature;    /* JNI字段描述符, 用来表示Java里调用的函数的参数和返回值类型 */
    void *fnPtr;          /* C语言实现的本地函数 */
} JNINativeMethod;
#endif

static jint fd;

jint msaOpen(JNIEnv *env, jobject cls)
{
	fd = open("dev/msa300", O_RDWR);
	
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "native open %d ", fd); //打印信息输出到android studio
//	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, strerror(errno)); //输出err log信息
	if(fd >= 0)
		return 0;
	else
		return -1;
}

jint msaInit(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_INIT,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function init() ret =  %d ", ret ); //打印信息输出到android studio
	return ret;
}

jint msaReadChipInfo(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_READ_CHIPINFO,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function readChipInfo() ret =  %d ", ret );
	return ret;
}

jint msaReadSensorData(JNIEnv *env, jobject cls, jintArray accData)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_READ_SENSORDATA,accData);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function readXYZ() ret = %d ", ret );
	return ret;
}

jint msaReadGain(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_READ_GAIN,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function readGain() ret =  %d ", ret );
	return ret;
}

jint msaReadRawData(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_READ_RAW_DATA,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function readRawData() ret =  %d ", ret );
	return ret;
}

jint msaSetCali(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_SET_CALI,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function setCali() ret =  %d ", ret );
	return ret;
}

jint msaGetCali(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_GET_CALI,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function getCali() ret =  %d " , ret );
	return ret;
}

jint msaClrCali(JNIEnv *env, jobject cls)
{
	int ret = ioctl(fd,GSENSOR_IOCTL_CLR_CALI,0);
	__android_log_print(ANDROID_LOG_DEBUG, MSA_TAG, "function clrCali() ret =  %d ", ret );
	return ret;
}

static const JNINativeMethod methods[] = {
	{"open", "()I", (void *)msaOpen},
	{"init", "()I", (void *)msaInit},
	{"readChipInfo", "()I", (void *)msaReadChipInfo},
	{"readSensorData", "([I)I", (void *)msaReadSensorData},
	{"readGain", "()I", (void *)msaReadGain},
	{"readRawData", "()I", (void *)msaReadRawData},
	{"setCali", "()I", (void *)msaSetCali},
	{"getCali", "()I", (void *)msaGetCali},
	{"clrCali", "()I", (void *)msaClrCali},
};

/* System.loadLibrary */
JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM *jvm, void *reserved)
{
	JNIEnv *env;
	jclass cls;

	if ((*jvm)->GetEnv(jvm, (void **)&env, JNI_VERSION_1_4)) { 
		return JNI_ERR; /* JNI version not supported */
	}
	cls = (*env)->FindClass(env, "com/thisway/hardlibrary/MsaControl");
	if (cls == NULL) {
		return JNI_ERR;
	}

	/* 2. map java hello <-->c c_hello */
	if ((*env)->RegisterNatives(env, cls, methods, sizeof(methods)/sizeof(methods[0])) < 0)
		return JNI_ERR;

	return JNI_VERSION_1_4;
}

