########################授权信息#######################
#授权状态
uint8 IICENSE_SUCCESS = 0x00                #许可证验证成功
uint8 LICENSE_INVALID = 0x01                #许可证无效
uint8 LODA_PUBLIC_KEY_FILE_ERR = 0x02       #加载公钥文件错误
uint8 INVALID_PUBLIC_KEY_ERR = 0x03         #公钥无效
uint8 LODA_SIGNATURE_FILE_ERR = 0x04        #加载签名文件错误
uint8 GET_SERIAL_NUMBER_ERR = 0x05          #获取序列号错误
uint8 LODA_PUBLIC_KEY_MEM_ERR = 0x06        #加载公钥内存错误

#授权信息
bool is_authorize           #是否授权
uint8 license_status        #授权状态
string info                 #信息
string serial_number        #当前序列号
string signature_path       #签名文件路径
string public_key           #当前公钥
string describe             #公钥描述