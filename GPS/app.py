import pynmea2
import requests
import json

from flask import Flask, render_template, request

app = Flask(__name__)

@app.route('/')
def index():

    return render_template('./index.html')

@app.route('/GPS',methods=['get', 'post'])
def GPS():
    text = request.form['text']
    print(text)

    #text = input('请输入$GNRMC相关信息：')
    # text = "$GNRMC,043835.000,A,2141.04929,N,11055.06852,E,11.52,161.85,200421,,,A*49"

    msg = pynmea2.parse(text)

    longitude = round(float(msg.longitude), 6)  # 经度
    latitude = round(float(msg.latitude), 6)  # 纬度

    location = str(longitude) + ',' + str(latitude)
    print("经纬度" + location)

    key = "6c2520fc9830fddf0c2aa1a4c9e84c81"

    url = f'https://restapi.amap.com/v3/geocode/regeo?key={key}&location={location}'
    r = requests.get(url)
    if r.status_code == 200:
        answer = r.json()
        json_data = json.loads(r.text)
        addr = json_data['regeocode']['formatted_address']
    else:
        pass

    print("地点：", addr)

    longitude1 = longitude
    latitude1 = latitude
    addr1 = addr
    print(longitude1 ,latitude1,addr1)
    return render_template('./gps.html',longitude = longitude,latitude = latitude,addr = addr)

if __name__ == '__main__':
    app.run()
