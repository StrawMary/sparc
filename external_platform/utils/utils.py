from weather import Weather, Unit
from datetime import datetime
from dateutil import parser
import datetime

# Auxiliary structure for 'time' subject.
class Dict(dict):
	def __getitem__(self, key):
		val = super(Dict, self).__getitem__(key)
		if callable(val):
			return val()
		return val


def get_weather_report(optional_parameters):
	if not 'datetime' in optional_parameters:
		return "Sorry, I don't know"
	date = parser.parse(optional_parameters['datetime'])
	weather = Weather(unit=Unit.CELSIUS)
	location = weather.lookup_by_location('bucharest')
	forecasts = location.forecast
	for forecast in forecasts:
		dt = parser.parse(forecast.date)
		if dt.day == date.day and dt.month == date.month and dt.year == date.year:
			return 'It will be ' + forecast.text + '. The maximum temperature is ' + forecast.high + ' degrees celsius'


def get_time_presentation(optional_parameters):
	time = datetime.now()
	return 'It is ' + str(time.hour) + ' ' + str(time.minute)


