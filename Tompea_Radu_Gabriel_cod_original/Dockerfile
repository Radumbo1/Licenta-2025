# Step 1: Use an official Python runtime as a parent image
FROM python:3.13
# Step 2: Set the working directory inside the container
WORKDIR /app

# Step 3: Copy the current directory contents into the container at /app
COPY . /app

# Step 4: Install any needed dependencies specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Step 5: Make the container's entry point the Python script
CMD ["python", "upload_data.py"]