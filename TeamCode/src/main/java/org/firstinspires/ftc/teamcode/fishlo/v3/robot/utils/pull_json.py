import pandas as pd
import numpy as np
import json
import os

json_file = "/org/firstinspires/ftc/teamcode/fishlo/v3/robot/utils/intermediate_json.json"
java_file = "/org/firstinspires/ftc/teamcode/fishlo/v3/robot/utils/PlaybackData.java"
df = pd.read_csv("~/Documents/record.csv")
new_data = json.loads(df.to_json(orient='records'))
data = dict()
data[r'PLAYBACK'] = list()

for element in new_data:
    data[r'PLAYBACK'].append(element)

with open(json_file, 'w') as f:
    json.dump(data, f, indent = 3)

with open(java_file, 'w') as f:
    f.write("import org.json.*;" + os.linesep)
    f.write("public class PlaybackData {" + os.linesep)
    f.write("private JSONParser parser = new JSONParser();" + os.linesep)
    f.write(f'private JSONObject obj = (JSONObject) parser.parse(new FileReader("{json_file}"));' + os.linesep)
    f.write('private JSONArray arr = (JSONArray) obj.get("PLAYBACK");' + os.linesep)
    f.write("public static final String d = arr.toString();" + os.linesep)
    f.write("}")