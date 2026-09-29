# iops-studio

## Prerequisites

- A GitHub account with access to this repository
- [Visual Studio Code](https://code.visualstudio.com/) with the **GitHub Codespaces** extension installed

## Getting started

1. In this repository, select **Code** (green button) → **Codespaces** tab.
2. Click **+** to create a new Codespace and note the name once it starts.
3. On your local machine, open **VS Code** → **Remote Explorer** → sign in to GitHub if prompted.
4. Find the Codespace you just created and select **Connect**.
5. A terminal will open with the install script downloading and installing `iops-studio` from Artifactory. **Do not close this terminal.** To monitor setup progress in a second terminal:

    ```bash
    tail -f /var/log/iops-studio/setup.log
    ```

6. In VS Code, open the **Ports** view and wait for **port 40128** to show a green status.
7. Once port 40128 is green, access the application at [http://localhost:40128](http://localhost:40128).

On subsequent starts or reconnects, services restart automatically.

## Troubleshooting

If the install script fails, verify your Codespace secrets are set correctly:

```bash
echo $ARTIFACTORY_USER   # should not be empty
```

To restart services after a Codespace reconnect if they didn't come up:

```bash
iops-studio --start
iops-studio --verify
```

## Local docs runtime

Start the documentation server with `docker compose up -d`. The Docker Compose definition lives in `compose.yaml` at the repository root.
