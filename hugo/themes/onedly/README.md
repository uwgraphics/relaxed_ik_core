# One-Day-Only (OneDly) Project Theme

This [Hugo](http://gohugo.io) theme is for documenting One-Day-Only (OneDly) projects. The theme renders a single page and is based on [Github project Page](https://themes.gohugo.io/github-project-landing-page/).

# About 

One-Day-Only or OneDly projects help to acquire a new skill or to complete a task within a short timeframe. It limits the available time and combines it with defined objectives. This situation forces you to focus on the relevant elements in order to successfully complete a task or project. Please note, the idea is not to pack as much work as you can do within a workday. At the end you will find answers for the following questions 

* What were the relevant elements for success?
* How good are you in estimating your work effort?
* What were road blocks or even showstoppers you haven't forseen even in a short-term project horizon?
* What are typical reoccuring interuptions and distractions during your work?
* What is your personal pace when working intensly on a task or project?

This will certainly generate some personal insights and improve your activities in future projects.

# Features

* Social media links as buttons at the top of the page
* Social media sharing links as buttons at the end of the page
* All posts on a single page as separated sections
* Post ordering by `sec` parameter in post's front matter, instead of date
* Imprint and GDPR examples pages 
* Examples for project structure included

# Demo and Examples

Have a look at the [demo](http://cdeck3r.com/OneDly-Theme/).

The demo uses examples files. They structure a project and are stored in the theme's `exampleSite/content` directory. The `exampleSite` directory also contains a `config.toml` with default parameter settings.

# Screenshot

![screenshot](https://github.com/cdeck3r/OneDly-Theme/blob/master/images/screenshot.png)

# Quick Start

1. install [Hugo](http://gohugo.io) 
1. Create a new site: `hugo new site myproject`
1. Change dir: `cd myproject`. We now assume this directory as _current_ dir
1. Add the OneDly theme: `git submodule add https://github.com/cdeck3r/OneDly-Theme.git themes/onedly`
1. Copy `themes/onedly/exampleSite/config.toml` into current dir 
1. Copy example files from `themes/onedly/exampleSite/content` in project's `content` directory.
1. Start server: `hugo server -D -t onedly` 

# Customization

Check [config.toml](https://github.com/cdeck3r/OneDly-Theme/blob/master/exampleSite/config.toml) for available configuration. Below is a description for each of them.

**Imprint & GDRP:** Both declarations are stored in `content/imprint-gdpr`. The links are only displayed, if the files exist. So, you do not have dead links.

### Project Specification Parameters

The project name is specified as the title of the site.
```
title = "OneDly Project"
```

The project description follows in the `params` section.
```
[params]
    description = "One-Day-Only (OneDly) project documentation."
```

Other variables within the `[params]` section are 

* `author_name` the project author's name.
* `author_url` link to the personal website of the project author.
* `project_url` link to project url, e.g. the project's github repo 

**Note:** `description` and `author_name` are used as the site's meta data for author and description within the html header.

Examples

```
    author_name = "cdeck3r"
    author_url = "//cdeck3r.com"
    project_url = "//github.com/cdeck3r/OneDly-Theme"
```

### Section Ordering

The project documentation may consists of several files indicating different documentation sections. In Hugo-Speak these sections are pages stored in separate `.md` files. All files need to be placed in `myproject/content/post` directory. An example project content structure can be found in `exampleSite/content`. 

Hugo uses the [content directory with the most pages](https://gohugo.io/functions/where/#mainsections) as source for displaying your documentation. You may explicitly set the `post` directory or any other directory containing your project documentation as well as add additional directories in the `config.toml`. In the example below the content sources from the `post` and the `other` directories.

```
[params]
    ...
    # optionally specify where the content is
    # mainSections = ["post", "other"] 
```

One may create a new section using `hugo new post/about.md`. It is recommended to start with an About section. In the file's front matter specify 

```
+++
title = "About"
date = "2019-02-28"
sec = 1
+++
```

The `sec` variable defines the position on the site. `sec = 1` lets the About section appear at the beginning right below the description and social media links. In the next section, created by the command above, you may specify a larger number, e.g. `sec = 4`. This positions this new section after the About section and before a section with `sec = 5` or larger. If you do not specify a `sec` variable ordering falls back to default ordering by the date sourced from the front matter. Information on ordering are found in [Hugo's documentation](https://gohugo.io/templates/lists/#default-weight-date-linktitle-filepath).


### Social media

A user may specify links to well-known social media sites shown at the top of the page. In `config.toml` the section `[social]` lists the supported social media links. If the variable in this section specifies the user's account name, the corresponding button linking to the social media site will appear on the page.

The exampleSite's [config.toml](https://github.com/cdeck3r/OneDly-Theme/blob/master/exampleSite/config.toml) for examples.

### Theme colors

Standard theme colors are 

```    
    first_color="#f8f8f8"
    first_border_color="#e7e7e7"
    first_text_color="#333"

    second_color="white"
    second_text_color="#333"

    header_color="#f8f8f8"
    header_text_color="rgb(51, 51, 51)"

    header_link_color="#777"
    header_link_hover_color="rgb(51, 51, 51)"

```

