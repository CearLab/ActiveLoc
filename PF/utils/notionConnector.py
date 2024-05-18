

import requests
import time
from dotenv import load_dotenv
import os
__all__ = ["add_experiment_to_notion"]
load_dotenv()
NOTION_SECRET = os.getenv("NOTION_SECRET")
NOTION_EXP_TABLE_ID = os.getenv("NOTION_EXP_TABLE_ID")

def add_experiment_to_notion(experiment_data):
    """
    Adds an experiment to a Notion database.

    Args:
        integration_token (str): The integration token for Notion API.
        database_id (str): The ID of the Notion database.
        experiment_data (dict): The data about the experiment to be added. Should be in the format:
            {
                "Name": "Experiment Name",
                "Description": "Experiment Description",
                "Date": "2023-05-17",
                ...
            }
    """
    experiment_data['Date'] = time.strftime("%Y-%m-%d")
    url = f"https://api.notion.com/v1/pages"
    
    headers = {
        "Authorization": f"Bearer {NOTION_SECRET}",
        "Content-Type": "application/json",
        "Notion-Version": "2022-06-28"
    }
    
    # Construct the properties for the new page
    properties = {
        "Name": {
            "title": [
                {
                    "text": {
                        "content": experiment_data.get("Name", "")
                    }
                }
            ]
        },
        "Description": {
            "rich_text": [
                {
                    "text": {
                        "content": experiment_data.get("Description", "")
                    }
                }
            ]
        },
        "Date": {
            "date": {
                "start": experiment_data.get("Date", "")
            }
        },
        "folder path": {
            "rich_text": [
                {
                    "text": {
                        "content": experiment_data.get("folder path", "")
                    }
                }
            ]
        },
        "params json": {
            "rich_text": [
                {
                    "text": {
                        "content": experiment_data.get("params", "")
                    }
                }
            ]
        }
        # Add other properties as needed
    }
    
    data = {
        "parent": {
            "database_id": NOTION_EXP_TABLE_ID
        },
        "properties": properties
    }
    
    response = requests.post(url, headers=headers, json=data)
    
    if response.status_code == 200:
        print("Experiment added successfully!")
    else:
        print(f"Failed to add experiment. Response: {response.text}")

if __name__ == "__main__":
    # Example usage
    experiment_data = {
        "Name": "Test Experiment",
        "Description": "This is a test experiment.",
        "Date": time.strftime("%Y-%m-%d"),
        "folder path": "sandbox/test_experiment"
    }
    add_experiment_to_notion(experiment_data)
