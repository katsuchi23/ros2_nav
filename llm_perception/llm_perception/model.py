import re
from langchain_community.chat_models import ChatOllama
from langchain.prompts import PromptTemplate

class Model:
    def __init__(self):
        self.model = ChatOllama(
            model="deepseek-r1:latest",
            temperature=0.1,
        )

        # Initialize conversation with a system prompt (for chatting)
        self.system_prompt = '''
            You are a helpful robot assistant that will decide where the robot should go based on the current position and several safety points' coordinates.
            You should decide which safety point is nearest to the robot and provide that safety point's coordinates as the output.
            There are 3 safety points with coordinates of (-1.98,0), (-0.3,-1.93), and (-0.3, 1.93).
            Please decide which safety point the robot should go to and give your output in the format of the coordinates of that safety point as well. 
            You MUST return the final answer and also make sure the final answer is within \\boxed{}.
            The format output is: \\boxed{(x,y)}. For example, \\boxed{(-1.98,0)}.
        '''

    def extract_boxed_text(self, text):
        pattern = r'\\boxed{(.*?)}'
        matches = re.findall(pattern, text)
        if not matches:
            return ""
        for match in matches[::-1]:
            if match != "":
                return match
        return "Coordinate is not found"
    
    def parse_coordinates(self, coord_str):
        # Remove parentheses and split by comma
        coord_str = coord_str.strip('()')
        x_str, y_str = coord_str.split(',')
        # Convert strings to floats and return as a tuple
        return float(x_str), float(y_str)

    def response(self, current_coordinate):
        prompt = PromptTemplate(
            input_variables=["system_prompt", "current_coordinate"],
            template="""
            System:
            {system_prompt}

            Current Position of the robot:
            {current_coordinate}

            Response:
            """
        )

        final_prompt = prompt.format(
            system_prompt=self.system_prompt,
            current_coordinate=current_coordinate
        )

        response = self.model.invoke(final_prompt).content
        print(response)
        coordinate = self.extract_boxed_text(response)
        coordinate = self.parse_coordinates(coordinate)
        return coordinate

if __name__ == "__main__":
    model = Model()
    current_coordinate = (-0.5, 1.7)
    print(f'Current Robot Coordinate is x:{current_coordinate[0]} y:{current_coordinate[1]}')
    ans = model.response(current_coordinate)
    print(ans)
    # print(f'Navigate to safety point x:{current_coordinate[0]} y:{current_coordinate[1]}')
