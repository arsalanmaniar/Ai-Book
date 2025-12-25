import React from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

// Hardcoded blog highlights data based on existing blog posts
const blogHighlightsData = [
  {
    title: "Physical AI Trends in 2025",
    description: "Exploring the latest advancements in Physical AI and how they're shaping the future of robotics and autonomous systems.",
    permalink: "/blog/physical-ai-trends",
    date: "2025-01-01"
  },
  {
    title: "Humanoid Robotics: The Next Frontier",
    description: "Insights into the convergence of AI and human-like interaction in next-generation humanoid robots.",
    permalink: "/blog/humanoid-robotics-insights",
    date: "2025-01-02"
  },
  {
    title: "Learning Reflections: Physical AI & Robotics",
    description: "A student's journey through the fundamentals of Physical AI and Robotics, with key insights and learning outcomes.",
    permalink: "/blog/learning-reflections",
    date: "2025-01-03"
  }
];

const BlogHighlights: React.FC = () => {
  return (
    <section className={styles.blogHighlightsSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <h2 className={styles.sectionTitle}>Latest Robotics Insights</h2>
          <p className={styles.sectionSubtitle}>
            Discover the latest trends and insights in Physical AI and Humanoid Robotics
          </p>
        </div>
        <div className={styles.blogHighlightsGrid}>
          {blogHighlightsData.map((post) => (
            <Link
              key={post.permalink}
              to={post.permalink}
              className={styles.blogHighlightCard}
            >
              <h3 className={styles.blogTitle}>{post.title}</h3>
              <p className={styles.blogExcerpt}>{post.description}</p>
              <div className={styles.blogMeta}>
                <time dateTime={post.date}>
                  {new Date(post.date).toLocaleDateString('en-US', {
                    year: 'numeric',
                    month: 'long',
                    day: 'numeric'
                  })}
                </time>
              </div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
};

export default BlogHighlights;