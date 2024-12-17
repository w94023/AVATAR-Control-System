using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using SimpleFileBrowser;

namespace UnityUI
{
    public class Browser : MonoBehaviour
    {
        public enum Mode
        {
            Folder,
            CSVFile,
        }

        public Text   text;
        public Button button;
        public Mode   mode = Mode.Folder;

        [Space(20)]

        public  string initialDirectory = "";
        private string _directory = ""; public string directory { set { _directory = value; if (text != null) text.text = _directory; } get { return _directory; } }

        public UnityEvent<string> onDirectoryChanged;

        private void Start()
        {
            FileBrowser.SetFilters( true, new FileBrowser.Filter( "Images", ".jpg", ".png" ), new FileBrowser.Filter( "Text Files", ".txt", ".pdf" ) );
            FileBrowser.SetDefaultFilter( ".jpg" );
            FileBrowser.SetExcludedExtensions( ".lnk", ".tmp", ".zip", ".rar", ".exe" );
            FileBrowser.AddQuickLink( "Users", "C:\\Users", null );
        }

        public void OpenFolderBrouser()
        {
            FileBrowser.ShowLoadDialog( ( paths ) => { OnDirectorySelected(paths[0]); },
								   () => { OnDirectoryUnselected(); },
								   FileBrowser.PickMode.Folders, false, null, null, "Select Folder", "Select" );
        }

        private void OnDirectorySelected(string directory)
        {
            this.directory = directory;
            onDirectoryChanged?.Invoke(directory);
        }

        private void OnDirectoryUnselected()
        {
            if (directory != "") {
                onDirectoryChanged?.Invoke(directory);
            }
            Clear();
        }

        public void Clear()
        {
            directory = "";
        }
    }
}

