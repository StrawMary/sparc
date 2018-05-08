

class EventGatherer:
    def __init__(self, url = "http://45.63.117.225:3000/Events"):
        self.url = url

    def get_events(self):
        import requests
        from requests.auth import HTTPDigestAuth
        import json

        # It is a good practice not to hardcode the credentials. So ask the user to enter credentials at runtime
        myResponse = requests.get(self.url)

        # For successful API call, response code will be 200 (OK)
        if (myResponse.ok):

            # Loading the response data into a dict variable
            # json.loads takes in only binary or string variables so using content to fetch binary content
            # Loads (Load String) takes a Json file and converts into python data structure (dict or list, depending on JSON)
            jData = json.loads(myResponse.content)
            return jData
        else:
            return []

            
#Example Call:
#eg = EventGatherer()
#print(eg.get_events())