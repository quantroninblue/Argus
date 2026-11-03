# What leaves your machine

Argus works with no model connected: validation, diagnosis, routes and driving all run on the
hub and the robot. This page covers what changes when you connect one.

## Local models

A connection to an address on this machine (`localhost`, `127.0.0.1`) keeps everything here.
`argus models list` shows such a connection as "this machine". Argus downloads no model weights;
you run the server yourself (Ollama, llama.cpp, vLLM, LM Studio).

## Remote models

With a remote connection, data goes to that provider only when you ask for something, under
their terms and retention rules. Argus sends nothing in the background, and robots never talk to a
model.

| Command | What is sent |
|---|---|
| `argus runs explain` | The run's outcome and findings, its timeline, the capability report and the last 80 lines of `argo.log` |
| `argus jobs explain` | Each run's outcome and findings |
| `argus design review` | The design's check results, its capability report, and the outcomes and findings of its jobs. URDF and mesh files are not sent |
| `argus say` | Your words and the names of the site's places and routes. Positions are not sent |
| `argus models test` | A short fixed prompt, and the image you pass with `--image` |

`argus jobs compare` uses no model. Connecting a provider also fetches its model list.

## Your own plans

`argus models connect chatgpt` signs in to your ChatGPT plan through OpenAI's route for open-source
apps. You sign in on OpenAI's page in your browser, on this machine, and OpenAI issues a client for
you on this hub. The commands above then go to OpenAI and count against your plan's limits. Requests
are made with `store: false`. Usage and limits are at <https://chatgpt.com/settings/usage>, which is
also where you can end the connection. OpenAI lists this route as a preview.

`argus models connect claude-code` runs the Claude Code program you have installed, as it is, under
your own sign-in. The commands above then go to Anthropic through that program and count against your
own usage, under Anthropic's terms for Claude Code. Argus never asks for, reads or stores that
sign-in, and it cannot sign you in; if the program is signed out, sign in with `claude` yourself. It
runs with no tools, no settings and no project files, in an empty directory, because run logs are
data from outside. Claude Code is Anthropic's product; Argus is not affiliated with Anthropic. If you
use Argus on behalf of other people, use an API key instead.

## Keys and tokens

API keys and ChatGPT tokens stay in the hub's memory and are never written to disk or logs. After a
hub restart, connect again with the key, or sign in again. `models.yaml` in the hub's home keeps
each connection's name, kind and address, which model fills each slot and, for ChatGPT, the client
OpenAI issued, your account's opaque id and a random id for this hub. It holds no email address.
Argus refuses to send a key over plain http to another machine and never follows a redirect with
one.

`argus models disconnect` ends a ChatGPT sign-in with OpenAI and says so if OpenAI did not confirm
it. The issued client is kept, so the same account signs back in as the same client.

## What a model cannot do

A model's answer is advice for you to read. It cannot change a profile, a speed limit, a safety
margin or a rule of the safety supervisor, and Argo never waits on one. Answers cite the measured
facts they rest on, and Argus warns when an answer cites none or cites facts that do not exist.
`argus say` can only pick a place or route the site already has, and asks you to confirm before
the robot moves.
