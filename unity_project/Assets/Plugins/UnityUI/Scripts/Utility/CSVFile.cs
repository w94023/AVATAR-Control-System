using System;
using System.IO;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEngine;

namespace UnityUI
{
    public static class CSVFile
    {
        public static string GetCurrentDirectory()
        {
            return Directory.GetCurrentDirectory();
        }

        public static void CheckDuplicatedFileName(string directory, ref string fileName)
        {
            if (File.Exists(directory + "\\" + fileName)) {
                AdjustFileName(ref fileName);
                CheckDuplicatedFileName(directory, ref fileName);
            }
        }

        public static void AdjustFileName(ref string fileName)
        {
            if (fileName.EndsWith(".csv", StringComparison.OrdinalIgnoreCase)) {
                // 파일명에서 ".csv" 제거
                fileName = fileName.Substring(0, fileName.Length - 4);
            }

            // 문자열이 _숫자로 끝나는 지 확인
            // 정규식을 사용하여 _숫자로 끝나는지 확인
            Match match = Regex.Match(fileName, @"_(\d+)$");
            if (match.Success) {
                // 문자열이 _숫자로 끝날 경우, 숫자를 1 더함
                string num = match.Groups[1].Value;
                int numLength = num.Length;
                int numCount = int.Parse(num);
                fileName = fileName.Substring(0, fileName.Length - (numLength + 1));
                fileName += "_" + (numCount + 1).ToString();
            }
            else {
                // 문자열이 _숫자로 끝나지 않을 경우, _1을 문자열 끝에 붙임
                fileName += "_1";
            }
            fileName += ".csv";
        }

        public static string[,] Read(string filename)
        {
            string filePath = filename;
            if (!File.Exists(filePath)) return null;

            List<string[]> csvData = new List<string[]>();
            string[] fileLines = File.ReadAllLines(filePath);

            int colNum = 0;
            for (int i = 0; i < fileLines.Length; i++) {
                string[] lineData = fileLines[i].Split(',');
                colNum = Math.Max(colNum, lineData.Length);
                csvData.Add(lineData);
            }
            int rowNum = csvData.Count;

            string[,] csvDataArr = new string[rowNum, colNum];
            for (int i = 0; i < rowNum; i++) {
                string[] lineData = csvData[i];
                for (int j = 0; j < lineData.Length; j++) {
                    csvDataArr[i, j] = lineData[j];
                }
            }

            return csvDataArr;
        }

        public static bool Write(string[,] data, string filename)
        {
            string filePath = filename;

            try {
                StreamWriter writer = new StreamWriter(filePath);
                for (int i = 0; i < data.GetLength(0); i++) {
                    string[] rowString = new string[data.GetLength(1)];
                    for (int j = 0; j < data.GetLength(1); j++) {
                        rowString[j] = data[i, j];
                    }
                    string rowStringJoined = string.Join(",", rowString);
                    writer.WriteLine(rowStringJoined);
                }
                writer.Close();
                return true;
            }
            catch{
                return false;
            }
        }

        /* Android */
        // private void Awake() {
        //     filePath = Path.Combine(Application.persistentDataPath);
        //     DeleteAllFilesInDirectory(filePath);

        //     stringBuilder = new StringBuilder();
        //     startTime = Time.realtimeSinceStartup;
        // }

        // public void StartRecording() {
        //     if (routine == null) {
        //         startTime = Time.realtimeSinceStartup;
        //         routine = StartCoroutine(SaveData());
        //     }
        // }

        // public void StopRecording() {
        //     if (routine != null) {
        //         StopCoroutine(routine);
        //         routine = null;

        //         Debug.Log("Start saving");
        //         try {
        //             if (!Directory.Exists(filePath)) {
        //                 Directory.CreateDirectory(filePath);
        //             }
        //         }
        //         catch (Exception e) {
        //             Debug.Log(e);            
        //         }
                
        //         Debug.Log("File path generated");

        //         try {
        //             StreamWriter outStream = File.CreateText(filePath+"/"+fileName+trial.ToString()+".csv");
        //             outStream.Write(stringBuilder);
        //             outStream.Close();
        //         }
        //         catch (Exception e) {
        //             Debug.Log(e);
        //         }

        //         // File.AppendAllText(filePath, stringBuilder.ToString());

        //         Debug.Log("Saving done");
        //         Debug.Log("File path : "+filePath);
        //         trial++;
        //     }
        // }

        // private IEnumerator SaveData()
        // {
        //     while (true) {
        //         float elapsedTime = Time.realtimeSinceStartup - startTime;
        //         Debug.Log(elapsedTime);
        //         stringBuilder.Append(elapsedTime);
        //         stringBuilder.Append(delimiter);
        //         // stringBuilder.Append(_SceneManager.power);
        //         // stringBuilder.Append(delimiter);
        //         // stringBuilder.Append(_SceneManager.powerControlTarget);
        //         // stringBuilder.Append(delimiter);
        //         // stringBuilder.Append(_SceneManager.voltage);
        //         // stringBuilder.Append(delimiter);
        //         // stringBuilder.Append(_SceneManager.current);
        //         // stringBuilder.Append(delimiter);
        //         stringBuilder.AppendLine();

        //         Debug.Log("Data appended");
        //         yield return new WaitForSeconds(0.001f);
        //     }
        // }

        // private void DeleteAllFilesInDirectory(string path)
        // {
        //     if (Directory.Exists(path))
        //     {
        //         string[] files = Directory.GetFiles(path);
        //         foreach (string file in files)
        //         {
        //             File.Delete(file);
        //         }

        //         Debug.Log("All files in directory deleted at path: " + path);
        //     }
        //     else
        //     {
        //         Debug.LogWarning("Directory does not exist at path: " + path);
        //     }
        // }
    }
}