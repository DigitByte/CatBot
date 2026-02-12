# Publishing CatBot to GitHub

This guide helps you publish CatBot professionally and keep the course materials clean and consistent.

## 1. Create the repository
- Name: `catbot` or `catbot-course`
- Visibility: public (recommended for students) or private
- Description (suggested):
  CatBot is a 12-week introductory robotics course built around a quadruped robot platform, with full simulation, control, sensing, and hardware build resources.

## 2. Initialize and push
From the CatBot folder:
```
git init
git add -A
git commit -m "Initial course release"
```
Then connect to GitHub:
```
git remote add origin https://github.com/DigitByte/CatBot
git branch -M main
git push -u origin main
```

## 3. Update references
Replace placeholder URLs in:
- `setup.md`
- `README.md`

## 4. Add course releases
Tag milestones so students can sync with the syllabus.
```
git tag -a v0.1-week4 -m "Week 4: Kinematics"
git tag -a v0.2-week8 -m "Week 8: Control"
git tag -a v1.0-week12 -m "Week 12: Capstone"
git push --tags
```

## 5. Recommended GitHub settings
- Enable Issues for student questions.
- Enable Discussions for course Q&A.
- Add a `LICENSE` file (BSD or MIT recommended).
- Add GitHub Releases for each milestone.

## 6. Optional: Docs site
If you want a course site, enable GitHub Pages:
- Source: `main` branch, `/docs` folder
- Add a landing page that links to `COURSE.md` and the notebooks.

