import opuslib
import opuslib.api.encoder
import opuslib.api.decoder
#导入库

rwalist = [i for i in range(0,160)]#创建一个原始样本

enc = opuslib.Encoder(fs = 16000,channels = 1 ,application = "audio")#创建编码器
dec = opuslib.Decoder(fs = 16000,channels = 1 )#创建解码器
enc._set_vbr(1)
encoutput = enc.encode(bytes(rwalist),160)#开始编码，数据保存在encoutput中
decoutput = dec.decode(encoutput,160)#开始解码，数据保存在decoutput中

print("原始数据：",rwalist)
print("编码数据：",encoutput)
print("解码数据：",decoutput)