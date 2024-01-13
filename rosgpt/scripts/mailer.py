#!/usr/bin/env python
from text_to_speech_tests import eleven_labs_tts, pyttxx3_test
import rospy
import speech_recognition as sr
import time
import sqlite3
from email.message import EmailMessage
import smtplib

EMAIL_ADDRESS = "mail"
EMAIL_PASSWORD = "password"


def insertVaribleIntoTable(id, name, email):
    try:
        dbase = sqlite3.connect('Our_data.db')
        cursor = dbase.cursor()
        print("Connected to SQLite")

        sqlite_insert_with_param = """INSERT INTO employee_email
                          (id, name, email) 
                          VALUES (?, ?, ?);"""

        data_tuple = (id, name, email)
        cursor.execute(sqlite_insert_with_param, data_tuple)
        dbase.commit()
        print("Python Variables inserted successfully into employee_email table")

        cursor.close()

    except sqlite3.Error as error:
        print("Failed to insert Python variable into sqlite table", error)
    finally:
        if dbase:
            dbase.close()
            print("The SQLite connection is closed")

def getDeveloperInfo(name):
    try:
        dbase = sqlite3.connect('Our_data.db')
        cursor = dbase.cursor()
        print("Connected to SQLite")

        sql_select_query = """select * from employee_email where name = ?"""
        cursor.execute(sql_select_query, (name,))
        records = cursor.fetchone()
        #print("Printing email:", records[2])
        cursor.close()

    except sqlite3.Error as error:
        print("Failed to read data from sqlite table", error)
    finally:
        if dbase:
            dbase.close()
            print("The SQLite connection is closed")
            return records[2]


def listen():
        r = sr.Recognizer()
        with sr.Microphone() as source:
            audio=r.listen(source)
            query = ''
            try:
                r.adjust_for_ambient_noise(source, duration=0.2)
                query = r.recognize_google(audio, language='en-US')
                return query
            except Exception as e:
                print("[ERROR] Invalid text, try again " )
        time.sleep(2)

def get_attributes():
    '''
    you can convert this node to speech input. Issue: wake word detection node is already running speech recognition which might cause an error
    '''
    #pyttxx3_test("To whom do you want to send an email?")
    #recepient = listen()
    #pyttxx3_test("What should be the date of appointment?")
    #date = listen()
    recepient = input("Recepient name: ")
    date = input("date of appointment: ")
    return recepient, date

def send_mail(email_id, context):

    msg = EmailMessage()
    msg['Subject'] = 'Ohmni Appointment Request'
    msg['From'] = EMAIL_ADDRESS
    msg['To'] = email_id

    msg.set_content(context)
    with smtplib.SMTP_SSL('smtp.gmail.com', 465) as smtp:
        smtp.login(EMAIL_ADDRESS, EMAIL_PASSWORD)
        smtp.send_message(msg)

def run_email_script():
    username = "Utsav"
    user_email = "hello@mail.me"
    dbase = sqlite3.connect('Our_data.db')
    print('Database opened')

    dbase.execute(''' CREATE TABLE IF NOT EXISTS employee_email(
     id, name, email) ''')
    insertVaribleIntoTable(1, 'UTSAV', 'panchalutsav274@gmail.com')
    insertVaribleIntoTable(2, 'NASIRU ABOKI', 'nasiru.aboki@iaas.uni-stuttgart.de')

    recepient, date = get_attributes()
    print(f"Received inputs: {recepient} {date}")
    recepient_email = getDeveloperInfo(recepient.upper())
    context = "Hello " + recepient_email + " this is appointment scheduler messege from Ohmni, "+ username + " requested to set \
                an appointment with you on " + date + " kindly respond with suitable slot to: " + user_email

    try:
        send_mail(recepient_email, context)
        pyttxx3_test(f"Email sent to {recepient} successfully")
    except Exception as e:
        print(f"[ERROR occured]{e}")

if __name__ == '__main__':
    run_email_script()