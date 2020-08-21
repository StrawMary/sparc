from calendar import TimeEncoding, month_name
from datetime import datetime
from dateutil import parser

import requests
import sys

api_key = "de6d52c2ebb7b1398526329875a49c57"


# Auxiliary structure for 'time' subject.
class Dict(dict):
    def __getitem__(self, key):
        val = super(Dict, self).__getitem__(key)
        if callable(val):
            return val()
        return val


def get_weather_report(optional_parameters):
    city = "Bucharest"
    date = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
    if "location" in optional_parameters:
        city = optional_parameters["location"]["value"]
    if 'datetime' in optional_parameters:
        date = optional_parameters['datetime']
    date = parser.parse(date)

    url = "https://api.openweathermap.org/data/2.5/forecast?q={}&units=metric&appid={}".format(city, api_key)
    response = requests.get(url).json()
    if "list" in response:
        try:
            for forecast in response["list"]:
                dt = parser.parse(forecast["dt_txt"])
                if dt.day == date.day and dt.month == date.month and dt.year == date.year and dt.hour == 15:
                    weather = forecast
                    with TimeEncoding("en_US.UTF-8") as encoding:
                        month = month_name[date.month]
                        description = weather["weather"][0]["description"]
                        temperature = weather["main"]["temp_max"]

                        return 'On %s %s it will be %s. The maximum temperature is %s degrees celsius' % \
                               (date.day, month, description, temperature)

        except Exception, e:
            return "Sorry, I don't know."
    return "Sorry, I can tell you only for the next 4 days."


def get_time_presentation(optional_parameters):
    time = datetime.now()
    return 'It is ' + str(time.hour) + ' ' + str(time.minute)


if __name__ == "__main__":
    print(get_weather_report({}))