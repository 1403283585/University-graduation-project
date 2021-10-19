'''
$GPRMC,024813.640,A,3158.4608,N,11848.3737,E,10.05,324.27,150706,,,A*50
字段0：$GPRMC，语句ID，表明该语句为Recommended Minimum Specific GPS/TRANSIT Data（RMC）推荐最小定位信息
字段1：UTC时间，hhmmss.sss格式
字段2：状态，A=定位，V=未定位
字段3：纬度ddmm.mmmm，度分格式（前导位数不足则补0）
字段4：纬度N（北纬）或S（南纬）
字段5：经度dddmm.mmmm，度分格式（前导位数不足则补0）
字段6：经度E（东经）或W（西经）
字段7：速度，节，Knots
字段8：方位角，度
字段9：UTC日期，DDMMYY格式
字段10：磁偏角，（000 - 180）度（前导位数不足则补0）
字段11：磁偏角方向，E=东W=西
字段12：模式，A=自动，D=差分，E=估测，N=数据无效（3.0协议内容）
字段13：校验值（$与*之间的数异或后的值）
'''

import pynmea2
import requests
import pprint
import json

from geopy.geocoders import Nominatim
from pyecharts.charts import Geo
from pyecharts import options
from pyecharts.globals import GeoType



#addr = input('请输入地点：')

#text = input('请输入$GNRMC相关信息：')

#text = "$GNRMC,074733.00,A,2241.28818,N,11358.44210,E,3.866,,070720,,,A*60"#深圳
#text = "$GNRMC,043835.000,A,2141.04929,N,11055.06852,E,11.52,161.85,200421,,,A*49"
longitude = input('经度：')
latitude = input('纬度：')
#msg = pynmea2.parse(text)
#print(msg.latitude,msg.longitude)

#longitude = round(float(msg.longitude),6) #经度
#latitude = round(float(msg.latitude),6) #纬度

longitude = round(float(longitude),6) #经度
latitude = round(float(latitude),6) #纬度


location=str(longitude)+','+str(latitude)

key="6c2520fc9830fddf0c2aa1a4c9e84c81" 

url = f'https://restapi.amap.com/v3/geocode/regeo?key={key}&location={location}'
r = requests.get(url)
if r.status_code == 200:
    answer = r.json()   
    json_data = json.loads(r.text)
    addr = json_data['regeocode']['formatted_address']
else:
    pass

print("地点：",addr)

'''
url2 = f'https://restapi.amap.com/v3/staticmap?key={key}&zoom={5}&size=750*300&location={location}'
r = requests.get(url2)
'''


