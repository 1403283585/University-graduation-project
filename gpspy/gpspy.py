import pynmea2
import requests
import pprint
import json

from geopy.geocoders import Nominatim
from pyecharts.charts import Geo
from pyecharts import options
from pyecharts.globals import GeoType

text = input('请输入$GNRMC相关信息：')

#text = "$GNRMC,074733.00,A,2241.28818,N,11358.44210,E,3.866,,070720,,,A*60"#深圳
#text = "$GNRMC,043835.000,A,2141.04929,N,11055.06852,E,11.52,161.85,200421,,,A*49"

msg = pynmea2.parse(text)
print(msg.latitude,msg.longitude)

longitude = round(float(msg.longitude),6) #经度
latitude = round(float(msg.latitude),6) #纬度

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



